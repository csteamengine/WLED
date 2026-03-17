#include "wled.h"
#include <math.h>

/*
 * Motor Controller Usermod (WLED)
 * - BTS7960 dual half-bridge motor driver (RPWM/LPWM + R_EN/L_EN)
 * - Momentary button start/stop toggle (to GND, internal pull-up)
 * - Quadrature hall sensors for position tracking & direction detection
 * - BTS7960 IS pin current sensing to detect stall/endstop by current spike
 * - Configurable target travel distance with auto-stop
 * - Bottom endstop switch for homing & absolute position tracking
 * - Firmware travel limit (30" / 762mm) to protect linear actuator stroke
 * - Homing sequence: motor must find bottom endstop before allowing upward travel
 * - Safe boot: motor never moves automatically on power-up
 * - Auto-homing: on first button press when unhomed, seeks bottom via stall, backs off, sets home
 *
 * Updates:
 * - Lower default PWM frequency (1kHz)
 * - S-curve acceleration/deceleration for gentle motor control
 * - Optional kickstart to overcome static friction
 * - Hall sensor direction detection (determines actual spin direction)
 * - Configurable distance per hall tick and target travel distance
 * - Bottom endstop with immediate stop (no decel ramp on endstop hit)
 * - Absolute position tracking from home (mm from bottom endstop)
 * - Travel limit enforcement: firmware hard limit at 457.2mm (18")
 * - User-adjustable max travel clamped to firmware limit
 * - Homing required before upward movement (safety against unknown position)
 * - Home command via JSON API, MQTT, and UI button
 * - FIX for ESP32 "dangerous relocation: l32r" linker errors:
 *     Use a global free-function ISR + global volatile snapshot variables
 *     (avoid C++ member functions / instance access inside IRAM ISR)
 * - FIX: acceleration not working:
 *     Do NOT overwrite PWM with pwmMax in RUNNING state (we maintain currentPwm)
 * - Replaced INA219 with BTS7960 IS pin current sensing via ESP32 ADC
 *     Reads both R_IS and L_IS, uses active half-bridge (max of both)
 *     Configurable sense resistor and IS ratio for calibration
 */

#if defined(ARDUINO_ARCH_ESP32)
  #include "soc/gpio_struct.h"
#endif

// -----------------------------------------------------------------------------
// IRAM-safe Hall ISR Globals (free function ISR avoids linker relocation issues)
// -----------------------------------------------------------------------------
static volatile int8_t  g_hallLastDelta  = 0;  // Last direction detected: +1 forward, -1 reverse, 0 no change
static volatile int32_t g_hallDeltaAccum = 0;  // Accumulated quadrature deltas from ISR
static volatile uint8_t g_hallLastAB     = 0;  // Last AB state used by ISR decoder
static volatile bool    g_hallInit       = false;
static uint8_t          g_hallAPin       = 32;
static uint8_t          g_hallBPin       = 35;

// -----------------------------------------------------------------------------
// IRAM-safe Endstop ISR Globals
// When the endstop fires, the ISR immediately kills PWM (zero latency)
// and sets a flag for loop() to handle state-machine cleanup.
// -----------------------------------------------------------------------------
static volatile bool g_endstopISRFired   = false;  // Set by ISR, cleared by loop()
static volatile bool g_endstopMotorArmed = false;   // true when motor is moving down (loop sets this)
static bool          g_endstopActiveLow  = true;
static uint8_t       g_endstopPin        = 22;
static uint8_t       g_rpwmChannel       = 4;
static uint8_t       g_lpwmChannel       = 5;
static uint8_t       g_renPin            = 25;
static uint8_t       g_lenPin            = 13;

#if defined(ARDUINO_ARCH_ESP32)
static inline uint8_t IRAM_ATTR read_gpio_pin_iram(uint8_t pin) {
  if (pin < 32) return (uint8_t)((GPIO.in >> pin) & 0x1);
  if (pin < 40) return (uint8_t)((GPIO.in1.data >> (pin - 32)) & 0x1);
  return 0;
}
#endif

void IRAM_ATTR hall_isr_capture() {
  uint8_t ab;
#if defined(ARDUINO_ARCH_ESP32)
  // Read from low/high GPIO input register based on pin number.
  const uint8_t a = read_gpio_pin_iram(g_hallAPin);
  const uint8_t b = read_gpio_pin_iram(g_hallBPin);
  ab = (a << 1) | b;
#else
  // Fallback (not IRAM-safe on non-ESP32), but keeps code portable.
  ab = ((uint8_t)digitalRead(g_hallAPin) << 1) | (uint8_t)digitalRead(g_hallBPin);
#endif

  if (!g_hallInit) {
    g_hallLastAB = ab;
    g_hallInit = true;
    return;
  }

  // Decode every transition in ISR and accumulate to avoid dropped-edge drift.
  const uint8_t transition = (uint8_t)((g_hallLastAB << 2) | ab);
  int8_t delta = 0;
  switch (transition) {
    case 0x2:  // 00 -> 10
    case 0x4:  // 01 -> 00
    case 0xB:  // 10 -> 11
    case 0xD:  // 11 -> 01
      delta = 1;
      break;
    case 0x1:  // 00 -> 01
    case 0x7:  // 01 -> 11
    case 0x8:  // 10 -> 00
    case 0xE:  // 11 -> 10
      delta = -1;
      break;
    default:
      break;
  }

  g_hallLastAB = ab;
  if (delta != 0) {
    g_hallDeltaAccum += delta;
    g_hallLastDelta = delta;
  }
}

// Endstop ISR: immediately kills motor PWM when endstop triggers.
// State-machine cleanup happens in loop() when it sees g_endstopISRFired.
void IRAM_ATTR endstop_isr_handler() {
  // Read pin via register for IRAM safety
  bool triggered;
#if defined(ARDUINO_ARCH_ESP32)
  uint8_t level = read_gpio_pin_iram(g_endstopPin);
  triggered = g_endstopActiveLow ? (level == 0) : (level != 0);
#else
  triggered = digitalRead(g_endstopPin) == (g_endstopActiveLow ? LOW : HIGH);
#endif

  if (triggered && g_endstopMotorArmed) {
    // Kill PWM and active brake — zero latency motor stop.
    // Keep enables HIGH to short the motor for regenerative braking.
#if defined(ARDUINO_ARCH_ESP32)
    ledcWrite(g_rpwmChannel, 0);
    ledcWrite(g_lpwmChannel, 0);
#endif
    digitalWrite(g_renPin, HIGH);
    digitalWrite(g_lenPin, HIGH);
    g_endstopMotorArmed = false;
    g_endstopISRFired = true;
  }
}

// Must be defined BEFORE the class
#ifndef USERMOD_ID_MOTOR_CONTROLLER
#define USERMOD_ID_MOTOR_CONTROLLER 9042
#endif

class MotorControllerUsermod : public Usermod {
private:
  // ---------------------------
  // Pin Configuration (BTS7960 motor driver)
  // ---------------------------
  int8_t rpwmPin = 27;   // BTS7960 RPWM — PWM for extend direction
  int8_t lpwmPin = 14;   // BTS7960 LPWM — PWM for retract direction
  int8_t renPin  = 25;   // BTS7960 R_EN — enable extend half-bridge (avoid GPIO12 strapping pin)
  int8_t lenPin  = 13;   // BTS7960 L_EN — enable retract half-bridge
  int8_t buttonPin = 33;  // Momentary button input (to GND, internal pull-up)

  // Bottom endstop switch
  int8_t endstopPin = 22;          // Endstop input
  bool   endstopActiveLow = true;  // true = LOW when triggered (pull-up + NO switch to GND)
  bool   endstopEnabled = true;    // Enable endstop functionality
  unsigned long endstopDebounceMs = 20;  // Debounce time for mechanical switch (ms)
  float bottomApproachMm = 50.0f;        // Start decelerating this far above bottom (mm)
  unsigned long lastEndstopChangeTime = 0;  // Last time raw endstop reading changed
  bool debouncedEndstopState = false;       // Debounced endstop triggered state
  bool lastRawEndstopState = false;         // Previous raw reading for edge detection

  // Hall sensors (quadrature)
  int8_t hallAPin  = 32;  // Hall A input (encoder channel A)
  int8_t hallBPin  = 35;  // Hall B input (encoder channel B, input-only OK)

  // BTS7960 current sense pins (IS outputs → resistor to GND → ADC)
  int8_t rIsPin = 39;  // R_IS analog input (ADC1, GPIO39/VN)
  int8_t lIsPin = 36;  // L_IS analog input (ADC1, GPIO36/VP)

  // ---------------------------
  // Timing / Motion Parameters
  // ---------------------------
  unsigned long accelTimeMs = 1500;
  unsigned long decelTimeMs = 800;

  // Run until spike OR user stops, but keep a safety timeout
  unsigned long safetyMaxRunMs = 20000; // 20s safety timeout

  // ---------------------------
  // Speed Parameters
  // ---------------------------
  int pwmMin = 0;
  int pwmMax = 255;
  int pwmChannelR = 4;  // LEDC channel for RPWM (extend) — use 4+ to avoid WLED conflicts
  int pwmChannelL = 5;  // LEDC channel for LPWM (retract)

  // Lower default frequency for many DC motor drivers
  int pwmFrequency = 1000; // was 5000
  int pwmResolution = 8;

  // Tracks which half-bridge is active (after inversion applied)
  bool activeDirectionForward = true;

  // Kickstart (helps overcome static friction / gearbox stiction)
  bool kickstartEnabled = false;
  int  kickstartPwm = 255;
  unsigned long kickstartMs = 120;

  // ---------------------------
  // Button debounce
  // ---------------------------
  unsigned long buttonDebounceMs = 80;
  unsigned long buttonLockoutMs = 300;

  // ---------------------------
  // Current sensing (BTS7960 IS pins via ADC)
  // ---------------------------
  // IS pin circuit: IS_pin → resistor to GND → ADC reads voltage across resistor
  // Motor current (mA) = ADC_voltage_mV * isRatio / isResistorOhms
  float isResistorOhms = 4700.0f;  // Sense resistor value (4.7kΩ default)
  float isRatio        = 8500.0f;  // BTS7960 current sense ratio (typ ~8500:1, calibrate per batch)
  bool  currentSenseEnabled = true; // Enable current sensing via IS pins
  uint8_t adcSampleCount = 4;      // Number of ADC samples to average (ESP32 ADC is noisy)

  // Polling interval (ms)
  unsigned long currentPollMs = 50;
  unsigned long lastCurrentPoll = 0;

  // Spike detection
  float currentSpikeThresholdmA = 3500.0f; // adjust for your actuator + load
  uint8_t spikeSamplesRequired  = 3;       // consecutive samples over threshold
  uint8_t spikeSampleCount      = 0;

  // Exposed telemetry
  float lastCurrentmA = 0.0f;

  // ---------------------------
  // Position tracking (hall quadrature)
  // ---------------------------
  volatile int32_t positionTicks = 0;

  // Distance configuration
  float ticksPerMm = 1.0f;             // Hall ticks per millimeter
  float targetDistance = 457.2f; // Target distance to travel (default max travel)
  bool  targetDistanceEnabled = true;            // Enable auto-stop at target distance

  // Starting position for current run (to calculate distance traveled)
  int32_t runStartTicks = 0;

  // Detected direction from hall sensors (actual motor spin direction)
  // +1 = forward, -1 = reverse, 0 = stationary/unknown
  int8_t detectedDirection = 0;

  // Smoothing for direction detection (consecutive same-direction ticks)
  uint8_t directionConfidence = 0;
  static const uint8_t DIRECTION_CONFIDENCE_THRESHOLD = 2;

  // ---------------------------
  // Homing & Absolute Position
  // ---------------------------
  bool isHomed = false;              // Has the motor found its home (bottom endstop)?
  float currentPositionMm = 0.0f;   // Absolute position in mm from home (0 = at endstop)

  // Direction mapping: which motor direction is "down" (toward endstop)?
  // false = reverse (motorDirection=false) is down; true = forward (motorDirection=true) is down
  bool downIsForward = false;

  // Invert motor direction: swaps RPWM/LPWM logic so "forward" spins the opposite way.
  // Use this if the motor runs backwards from expected without re-wiring.
  bool invertMotorDirection = false;

  // ---------------------------
  // Stall Detection (encoder-based, Safety Layer 2)
  // ---------------------------
  bool stallDetectionEnabled = true;      // Enable stall detection via encoder timeout
  unsigned long stallTimeoutMs = 150;     // No encoder pulse in this time = stalled (ms)
  unsigned long stallStartGraceMs = 100;  // Grace period after start before stall checks
  unsigned long lastHallTickTime = 0;     // Last time a hall encoder tick was processed
  bool hallSeenThisRun = false;           // True after first hall tick in current run

  // ---------------------------
  // Travel Limits
  // ---------------------------
  // 30 inches = 762.0mm — firmware hard limit (actuator has 35" stroke, leave margin
  // since "Set Bottom" may not be at 0" extended)
  static constexpr float FIRMWARE_MAX_TRAVEL_MM = 762.0f;
  float maxTravelDistance = 762.0f;  // User-adjustable max travel, clamped to firmware limit

  // ---------------------------
  // Motor Motion State Machine
  // ---------------------------
  enum MotorState { IDLE, STARTING, RUNNING, STOPPING };
  MotorState motorState = IDLE;

  bool motorDirection = true; // true = forward/up, false = reverse/down
  bool lastActiveMovingDown = false; // Snapshot of isMovingDown() before direction toggle (for coast tracking)
  bool manualJogActive = false; // true while UI hold-to-jog controls are active
  bool homeSeekActive = false;  // true only while explicit HOME command is seeking endstop

  // Auto-homing: on first button press when unhomed, seek bottom by stalling, then back off
  bool autoHomingActive = false;        // Auto-homing sequence in progress
  bool autoHomingPhaseBackoff = false;  // false = seeking down, true = backing off up
  int  homingPwmMax = 100;             // Slow PWM for auto-homing seek (0-255)
  float homingBackoffMm = 2.0f;       // Distance to back up after finding bottom (mm)
  unsigned long runStartTime = 0;

  // ramp controller
  unsigned long rampStartTime = 0;
  int rampStartPwm = 0;
  int rampTargetPwm = 0;
  int currentPwm = 0;

  // kickstart tracking (non-blocking)
  bool kickActive = false;
  unsigned long kickStartTime = 0;

  enum StopReason { STOP_USER, STOP_SPIKE, STOP_TIMEOUT, STOP_TARGET_REACHED, STOP_ENDSTOP, STOP_TRAVEL_LIMIT, STOP_STALL };
  StopReason lastStopReason = STOP_USER;

  // Button detection state
  bool buttonDetected = false;
  unsigned long lastButtonTime = 0;
  unsigned long buttonStartTime = 0;

  // Enable/disable usermod
  bool enabled = true;
  bool initDone = false;

  // ---------------------------
  // LED Control (based on lid state)
  // ---------------------------
  bool ledControlEnabled = true;     // Enable LED on/off based on motor direction
  bool ledInvertDirection = false;   // false: forward=opening (LEDs on), true: forward=closing (LEDs off)
  uint8_t ledSavedBri = 128;         // Saved brightness to restore when turning LEDs back on
  float ledOffDistanceMm = 10.0f;    // LEDs turn off when position is within this distance of bottom (mm)
  bool bootLedApplied = false;       // Flag to defer LED init to first loop (after WLED loads boot preset)

  // ---------------------------
  // LED Control (turn on/off based on lid position)
  // ---------------------------
  void updateLedState() {
    if (!ledControlEnabled) return;

    // When homed, use position to determine LED state.
    // LEDs ON when shelf is above the "off" threshold, OFF when near bottom.
    bool lidIsOpen;
    if (isHomed) {
      lidIsOpen = (currentPositionMm > ledOffDistanceMm);
    } else {
      // Not homed — leave LEDs in their current state (don't touch them)
      return;
    }

    if (lidIsOpen) {
      // Lid is open - turn LEDs ON (restore saved brightness)
      if (bri == 0) {
        bri = ledSavedBri > 0 ? ledSavedBri : 128;
        stateUpdated(CALL_MODE_DIRECT_CHANGE);
      }
    } else {
      // Lid is closed - turn LEDs OFF (save current brightness first)
      if (bri > 0) {
        ledSavedBri = bri;
        bri = 0;
        stateUpdated(CALL_MODE_DIRECT_CHANGE);
      }
    }
  }

  // ---------------------------
  // MQTT publish (optional)
  // ---------------------------
  void publishHomeAssistantSensor() {
#ifndef WLED_DISABLE_MQTT
    if (WLED_MQTT_CONNECTED) {
      char topic[96];
      char buf[32];

      const char* st =
        (motorState == RUNNING)  ? "running" :
        (motorState == STARTING) ? "starting" :
        (motorState == STOPPING) ? "stopping" :
                                   "idle";

      sprintf_P(topic, PSTR("%s/motor_state"), mqttDeviceTopic);
      mqtt->publish(topic, 0, false, st);

      sprintf_P(topic, PSTR("%s/motor_direction"), mqttDeviceTopic);
      mqtt->publish(topic, 0, false, motorDirection ? "forward" : "reverse");

      // Detected direction from hall sensors
      sprintf_P(topic, PSTR("%s/motor_hall_direction"), mqttDeviceTopic);
      const char* hallDir =
        (detectedDirection > 0) ? "forward" :
        (detectedDirection < 0) ? "reverse" : "unknown";
      mqtt->publish(topic, 0, false, hallDir);

      sprintf_P(topic, PSTR("%s/motor_current_ma"), mqttDeviceTopic);
      dtostrf(lastCurrentmA, 0, 1, buf);
      mqtt->publish(topic, 0, false, buf);

      sprintf_P(topic, PSTR("%s/motor_position_ticks"), mqttDeviceTopic);
      ltoa((long)positionTicks, buf, 10);
      mqtt->publish(topic, 0, false, buf);

      // Distance traveled
      sprintf_P(topic, PSTR("%s/motor_distance"), mqttDeviceTopic);
      dtostrf(getDistanceTraveled(), 0, 2, buf);
      mqtt->publish(topic, 0, false, buf);

      sprintf_P(topic, PSTR("%s/motor_pwm"), mqttDeviceTopic);
      itoa(currentPwm, buf, 10);
      mqtt->publish(topic, 0, false, buf);

      // Homing / endstop state
      sprintf_P(topic, PSTR("%s/motor_homed"), mqttDeviceTopic);
      mqtt->publish(topic, 0, false, isHomed ? "true" : "false");

      if (endstopEnabled) {
        sprintf_P(topic, PSTR("%s/motor_endstop"), mqttDeviceTopic);
        mqtt->publish(topic, 0, false, isEndstopTriggered() ? "triggered" : "open");
      }

      sprintf_P(topic, PSTR("%s/motor_position_mm"), mqttDeviceTopic);
      dtostrf(currentPositionMm, 0, 1, buf);
      mqtt->publish(topic, 0, false, buf);

      sprintf_P(topic, PSTR("%s/motor_max_travel_mm"), mqttDeviceTopic);
      dtostrf(maxTravelDistance, 0, 1, buf);
      mqtt->publish(topic, 0, false, buf);
    }
#endif
  }

  // ---------------------------
  // Motor control primitives (BTS7960 dual half-bridge)
  // ---------------------------
  // BTS7960 driving: PWM goes on RPWM or LPWM depending on direction.
  // R_EN and L_EN are held HIGH when motor is active, LOW when idle.
  void setDirectionPins(bool forward) {
    // Apply motor direction inversion (software swap of RPWM/LPWM logic)
    activeDirectionForward = invertMotorDirection ? !forward : forward;
    // Actual PWM routing happens in applyPwm()
  }

  void applyPwm(int pwmValue) {
    pwmValue = constrain(pwmValue, 0, 255);
    currentPwm = pwmValue;

#if defined(ARDUINO_ARCH_ESP32)
    if (activeDirectionForward) {
      ledcWrite(pwmChannelR, pwmValue);  // RPWM active (extend)
      ledcWrite(pwmChannelL, 0);         // LPWM off
    } else {
      ledcWrite(pwmChannelR, 0);         // RPWM off
      ledcWrite(pwmChannelL, pwmValue);  // LPWM active (retract)
    }
#else
    // Fallback for non-ESP32 (basic digital direction + analog PWM)
    if (activeDirectionForward) {
      analogWrite(rpwmPin, pwmValue);
      analogWrite(lpwmPin, 0);
    } else {
      analogWrite(rpwmPin, 0);
      analogWrite(lpwmPin, pwmValue);
    }
#endif
  }

  // Cut all motor outputs (both PWM channels to 0, disable half-bridges)
  void coastMotor() {
    g_endstopMotorArmed = false;  // Disarm ISR before touching hardware
#if defined(ARDUINO_ARCH_ESP32)
    ledcWrite(pwmChannelR, 0);
    ledcWrite(pwmChannelL, 0);
#else
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
#endif
    currentPwm = 0;
    digitalWrite(renPin, LOW);
    digitalWrite(lenPin, LOW);
  }

  // Active brake: PWM channels to 0 but keeps both half-bridges ENABLED.
  // This shorts the motor windings through the low-side FETs, converting
  // kinetic energy to heat and stopping the motor much faster than coasting.
  // Essential for gravity-assisted loads where coasting = freewheeling downward.
  void brakeMotor() {
    g_endstopMotorArmed = false;
#if defined(ARDUINO_ARCH_ESP32)
    ledcWrite(pwmChannelR, 0);
    ledcWrite(pwmChannelL, 0);
#else
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
#endif
    currentPwm = 0;
    // Keep enables HIGH — this shorts the motor for regenerative braking
    digitalWrite(renPin, HIGH);
    digitalWrite(lenPin, HIGH);
  }

  // Enable both half-bridges (call before applying PWM)
  void enableMotorDriver() {
    digitalWrite(renPin, HIGH);
    digitalWrite(lenPin, HIGH);
  }

  // S-curve (smoothstep) for gentle acceleration/deceleration
  // Returns value between 0.0 and 1.0 using smoothstep function
  float smoothstep(float t) {
    // Clamp t to [0, 1]
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    // Smoothstep: 3t^2 - 2t^3 (ease-in-out)
    return t * t * (3.0f - 2.0f * t);
  }

  // Smoother S-curve using smootherstep (Ken Perlin's improved version)
  // 6t^5 - 15t^4 + 10t^3
  float smootherstep(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
  }

  int computeRampPwm(unsigned long elapsed, unsigned long duration, int fromPwm, int toPwm) {
    if (duration == 0) return toPwm;
    if (elapsed >= duration) return toPwm;

    // Calculate progress as 0.0 to 1.0
    float progress = (float)elapsed / (float)duration;

    // Apply S-curve for smooth acceleration
    float smoothProgress = smootherstep(progress);

    // Interpolate between start and target PWM
    return fromPwm + (int)((float)(toPwm - fromPwm) * smoothProgress);
  }

  // ---------------------------
  // Endstop & Direction Helpers
  // ---------------------------
  // Returns debounced endstop state. Call updateEndstopDebounce() every loop iteration.
  bool isEndstopTriggered() {
    if (!endstopEnabled) return false;
    return debouncedEndstopState;
  }

  // Update debounced endstop reading — must be called every loop iteration.
  // Uses "instant trigger, delayed release" pattern: the endstop registers
  // immediately on first activation (safety-critical), but requires the full
  // debounce period of stable "open" readings before it clears.
  void updateEndstopDebounce() {
    if (!endstopEnabled) return;
    bool rawState = digitalRead(endstopPin) == (endstopActiveLow ? LOW : HIGH);
    if (rawState != lastRawEndstopState) {
      lastRawEndstopState = rawState;
      lastEndstopChangeTime = millis();
    }
    if (rawState && !debouncedEndstopState) {
      // Trigger immediately on activation — no delay
      debouncedEndstopState = true;
    } else if (!rawState && debouncedEndstopState) {
      // Only release after stable "open" for the full debounce period
      if (millis() - lastEndstopChangeTime >= endstopDebounceMs) {
        debouncedEndstopState = false;
      }
    }
  }

  bool isMovingDown() {
    return downIsForward ? motorDirection : !motorDirection;
  }

  bool isMovingUp() {
    return !isMovingDown();
  }

  // Set motor direction to "down" (toward endstop)
  void setDirectionDown() {
    motorDirection = downIsForward;
  }

  // Set motor direction to "up" (away from endstop)
  void setDirectionUp() {
    motorDirection = !downIsForward;
  }

  // Immediate hard stop — cuts power with no deceleration ramp.
  // Used when hitting the endstop (don't keep pushing against physical stop).
  void immediateStop(StopReason reason, bool toggleDirectionAfter = false) {
    brakeMotor();

    lastStopReason = reason;
    lastActiveMovingDown = isMovingDown();  // Save before toggle/IDLE for coast tracking
    motorState = IDLE;
    kickActive = false;
    spikeSampleCount = 0;

    updateLedState();

    if (toggleDirectionAfter) {
      motorDirection = !motorDirection;
    }
    manualJogActive = false;
    homeSeekActive = false;
    autoHomingActive = false;
    autoHomingPhaseBackoff = false;

    publishHomeAssistantSensor();
    serializeConfigToFS();
  }

  // Auto-homing: transition from seek-down to backoff-up after finding bottom
  void beginAutoHomingBackoff() {
    brakeMotor();
    motorState = IDLE;
    kickActive = false;
    spikeSampleCount = 0;
    autoHomingPhaseBackoff = true;
    homeSeekActive = true;
    setDirectionUp();
    beginStart();
  }

  // Auto-homing: complete the sequence and set home position
  void completeAutoHoming() {
    brakeMotor();
    motorState = IDLE;
    kickActive = false;
    autoHomingActive = false;
    autoHomingPhaseBackoff = false;
    homeSeekActive = false;
    isHomed = true;
    currentPositionMm = 0.0f;
    positionTicks = 0;
    setDirectionUp();
    updateLedState();
    publishHomeAssistantSensor();
    serializeConfigToFS();
  }

  void beginStop(StopReason reason) {
    if (motorState == IDLE || motorState == STOPPING) return;

    lastStopReason = reason;
    motorState = STOPPING;
    rampStartTime = millis();
    rampStartPwm = currentPwm;
    rampTargetPwm = 0;

    kickActive = false;
    publishHomeAssistantSensor();
  }

  void finalizeStopAndToggleDirection() {
    // Coast — disable both half-bridges
    coastMotor();

    // Save direction before toggle so coast-down ticks are tracked correctly
    lastActiveMovingDown = isMovingDown();

    updateLedState();

    motorState = IDLE;

    // Always toggle direction — beginStart() handles safety overrides
    // (e.g., forcing DOWN when unhomed, redirecting at endstop)
    motorDirection = !motorDirection;
    manualJogActive = false;
    homeSeekActive = false;
    autoHomingActive = false;
    autoHomingPhaseBackoff = false;

    spikeSampleCount = 0;
    publishHomeAssistantSensor();
    serializeConfigToFS();
  }

  void beginStart() {
    // ---------------------------
    // Pre-start safety checks
    // ---------------------------
    if (endstopEnabled && !manualJogActive) {
      // Auto up/down requires homing first. User must jog to the desired
      // bottom position and press "Set Bottom" before auto movement is allowed.
      if (!isHomed && !homeSeekActive && !autoHomingActive) {
        return;  // Not homed — refuse auto movement
      }

      // If trying to go DOWN but endstop is already triggered,
      // redirect to UP and continue (don't silently consume the button press)
      if (isMovingDown() && isEndstopTriggered()) {
        isHomed = true;
        currentPositionMm = 0.0f;
        positionTicks = 0;
        setDirectionUp();
        // Fall through to start the motor going UP
      }

      // Don't start moving UP if at or beyond travel limit
      if (isHomed && isMovingUp() && currentPositionMm >= maxTravelDistance) {
        return;  // At travel limit, refuse to go further up
      }
    }

    motorState = STARTING;
    runStartTime = millis();
    spikeSampleCount = 0;
    lastHallTickTime = millis();  // Reset stall timer
    hallSeenThisRun = false;

    // Record starting position for distance tracking
    runStartTicks = positionTicks;

    // Reset detected direction at start
    detectedDirection = 0;
    directionConfidence = 0;

    // Enable BTS7960 half-bridges and set direction (PWM routing)
    enableMotorDriver();
    setDirectionPins(motorDirection);

    // Arm endstop ISR for immediate hardware-level stop when moving down
    if (endstopEnabled && isMovingDown()) {
      g_endstopMotorArmed = true;
    }

    // Update LEDs based on current position (will turn on if above threshold)
    updateLedState();

    // Start at pwmMin (note: some motors won't move until higher PWM)
    applyPwm(constrain(pwmMin, 0, 255));

    kickActive = false;
    rampStartTime = millis();
    rampStartPwm = currentPwm;
    rampTargetPwm = pwmMax;

    // Auto-homing uses reduced speed for safety
    if (autoHomingActive) {
      rampTargetPwm = constrain(homingPwmMax, pwmMin, pwmMax);
    }

    publishHomeAssistantSensor();
  }

  void startManualJog(bool moveUp) {
    if (motorState != IDLE) return;
    if (!moveUp && endstopEnabled && isEndstopTriggered()) {
      // Already at bottom endstop: refuse downward jog to avoid pushing into the stop.
      isHomed = true;
      currentPositionMm = 0.0f;
      positionTicks = 0;
      setDirectionUp();
      return;
    }
    manualJogActive = true;
    homeSeekActive = false;
    if (moveUp) setDirectionUp();
    else setDirectionDown();
    beginStart();
    if (motorState == IDLE) {
      // Start was blocked by safety checks; clear manual mode.
      manualJogActive = false;
    }
  }

  // Shared toggle behavior for physical button and Info panel button.
  void handleToggleAction() {
    if (motorState == IDLE) {
      // If not homed, first button press starts auto-homing:
      // move down slowly until stall/endstop, back off, set home.
      if (!isHomed && endstopEnabled) {
        autoHomingActive = true;
        autoHomingPhaseBackoff = false;
        homeSeekActive = true;
        setDirectionDown();
        beginStart();
        return;
      }
      homeSeekActive = false;
      beginStart();
    } else if (motorState == STOPPING) {
      // Already decelerating — cut power immediately on second press.
      finalizeStopAndToggleDirection();
    } else {
      if (manualJogActive) immediateStop(STOP_USER);
      else beginStop(STOP_USER);
    }
  }

  void updateStartSequence() {
    const unsigned long now = millis();
    const unsigned long elapsed = now - rampStartTime;
    const int pwm = computeRampPwm(elapsed, accelTimeMs, rampStartPwm, rampTargetPwm);
    applyPwm(pwm);

    if (elapsed >= accelTimeMs) {
      motorState = RUNNING;
      lastHallTickTime = millis();  // Reset stall timer — don't penalize ramp-up time
      publishHomeAssistantSensor();
    }
  }

  void updateStopSequence() {
    const unsigned long now = millis();
    const unsigned long elapsed = now - rampStartTime;

    int pwm = computeRampPwm(elapsed, decelTimeMs, rampStartPwm, 0);

    // When decelerating toward bottom, cap PWM to the approach zone limit.
    // This ensures the motor is near-zero speed when it reaches 0mm,
    // minimizing coast-down after the hard floor immediateStop.
    if (isHomed && !homeSeekActive && !manualJogActive &&
        isMovingDown() && bottomApproachMm > 0.0f &&
        currentPositionMm <= bottomApproachMm && currentPositionMm > 0.0f) {
      float fraction = currentPositionMm / bottomApproachMm;
      int approachPwm = pwmMin + (int)((float)(pwmMax - pwmMin) * fraction);
      if (approachPwm < pwmMin) approachPwm = pwmMin;
      if (pwm > approachPwm) pwm = approachPwm;
    }

    applyPwm(pwm);

    if (elapsed >= decelTimeMs) {
      finalizeStopAndToggleDirection();
    }
  }

  // ---------------------------
  // Button detection
  // ---------------------------
  bool buttonPressed() {
    const unsigned long now = millis();

    // Lockout prevents rapid re-triggers after a confirmed press
    if (now - lastButtonTime < buttonLockoutMs) return false;

    const bool pressed = digitalRead(buttonPin) == LOW;  // Active LOW (pull-up to GND)

    if (pressed) {
      if (!buttonDetected) {
        // First LOW reading — start debounce timer
        buttonStartTime = now;
        buttonDetected = true;
      }
      if (now - buttonStartTime >= buttonDebounceMs) {
        // Stable LOW for buttonDebounceMs — confirmed press
        lastButtonTime = now;
        buttonDetected = false;
        return true;
      }
    } else {
      // Button released — reset detection state
      buttonDetected = false;
    }

    return false;
  }

  // ---------------------------
  // Hall processing (decode in loop)
  // ---------------------------
  void processHallIfPending() {
    int32_t deltaAccum = 0;
    noInterrupts();
    deltaAccum = g_hallDeltaAccum;
    g_hallDeltaAccum = 0;
    interrupts();

    if (deltaAccum == 0) return;

    positionTicks += deltaAccum;
    lastHallTickTime = millis();  // Track for stall detection
    hallSeenThisRun = true;

    const int8_t deltaDir = (deltaAccum > 0) ? 1 : -1;
    const uint8_t deltaMag = (uint8_t)((deltaAccum > 255 || deltaAccum < -255) ? 255 : abs((int)deltaAccum));

    // Build confidence in detected direction
    if (deltaDir == detectedDirection) {
      directionConfidence = (uint8_t)min(255, (int)directionConfidence + (int)deltaMag);
    } else {
      directionConfidence = 1;
      detectedDirection = deltaDir;
    }

    // Update absolute position when homed.
    // Track during ALL states including IDLE so that coast-down ticks after
    // an immediate stop are captured — prevents cumulative drift.
    // During active states: use commanded direction (known to be correct).
    // During IDLE: use lastActiveMovingDown (saved before direction toggle)
    //   so coast-down ticks are applied in the correct direction.
    if (isHomed) {
      const int32_t deltaAbs = (deltaAccum < 0) ? -deltaAccum : deltaAccum;
      float tickDistanceMm = (float)deltaAbs / ticksPerMm;
      bool movingDown = (motorState != IDLE) ? isMovingDown() : lastActiveMovingDown;
      if (movingDown) {
        currentPositionMm -= tickDistanceMm;
      } else {
        currentPositionMm += tickDistanceMm;
      }
      // Do NOT clamp to 0 here — let position go negative so coast-down
      // ticks after braking are tracked honestly. The endstop ISR/polled check
      // resets to 0 when the physical endstop triggers. Clamping here would
      // eat coast ticks and cause cumulative downward drift each cycle.
    }
  }

  // Get the distance traveled since the start of the current run
  float getDistanceTraveled() {
    int32_t ticksDelta = positionTicks - runStartTicks;
    // Use absolute value since we care about distance magnitude
    if (ticksDelta < 0) ticksDelta = -ticksDelta;
    return (float)ticksDelta / ticksPerMm;
  }

  // Check if target distance has been reached
  bool hasReachedTargetDistance() {
    if (!targetDistanceEnabled || targetDistance <= 0.0f) return false;
    return getDistanceTraveled() >= targetDistance;
  }

  // ---------------------------
  // Current sensing (BTS7960 IS pins via ADC)
  // ---------------------------
  // Reads both R_IS and L_IS, takes the higher value (only active half-bridge outputs current).
  // Converts ADC voltage to motor current:  mA = (voltage_mV * isRatio) / isResistorOhms
  float readMotorCurrentmA() {
    // Average multiple ADC samples to reduce ESP32 ADC noise
    const uint8_t samples = (adcSampleCount > 0) ? adcSampleCount : 1;
    long sumR = 0, sumL = 0;
    for (uint8_t i = 0; i < samples; i++) {
      sumR += analogRead(rIsPin);
      sumL += analogRead(lIsPin);
    }
    const int rawR = (int)(sumR / samples);
    const int rawL = (int)(sumL / samples);
    const int raw = max(rawR, rawL);

    // ESP32 12-bit ADC: 0–4095 maps to 0–3300mV (with default 11dB attenuation)
    const float voltage_mV = (float)raw * 3300.0f / 4095.0f;

    // IS_current = voltage / resistor;  motor_current = IS_current * ratio
    // Combined: motor_current_mA = voltage_mV * isRatio / isResistorOhms
    return voltage_mV * isRatio / isResistorOhms;
  }

  bool pollCurrentAndCheckSpike() {
    if (!currentSenseEnabled) return false;

    const unsigned long now = millis();
    if (now - lastCurrentPoll < currentPollMs) return false;
    lastCurrentPoll = now;

    const float mA = readMotorCurrentmA();
    lastCurrentmA = mA;

    if (mA >= currentSpikeThresholdmA) {
      if (spikeSampleCount < 255) spikeSampleCount++;
    } else {
      spikeSampleCount = 0;
    }

    return (spikeSampleCount >= spikeSamplesRequired);
  }

public:
  void setup() {
    if (!enabled) return;

    // BTS7960 motor driver pins
    pinMode(rpwmPin, OUTPUT);
    pinMode(lpwmPin, OUTPUT);
    pinMode(renPin, OUTPUT);
    pinMode(lenPin, OUTPUT);

    // If GPIO12 is used as enable pin, keep it LOW at init to reduce strap-related boot issues.
    // Recommended: use a non-strapping GPIO for renPin (default is GPIO25).
    digitalWrite(renPin, LOW);
    digitalWrite(lenPin, LOW);
    delay(100);

    // Button input (momentary switch to GND, internal pull-up)
    if (buttonPin >= 34 && buttonPin <= 39) {
      pinMode(buttonPin, INPUT);       // No internal pull-up available; needs external
    } else {
      pinMode(buttonPin, INPUT_PULLUP);
    }

    // Endstop input (GPIO 34-39 don't have internal pull-ups on ESP32)
    if (endstopEnabled) {
      if (endstopPin >= 34 && endstopPin <= 39) {
        pinMode(endstopPin, INPUT);       // Needs external pull-up/pull-down
      } else {
        pinMode(endstopPin, INPUT_PULLUP); // Use internal pull-up
      }
      // Initialize debounce state from current reading
      bool rawState = digitalRead(endstopPin) == (endstopActiveLow ? LOW : HIGH);
      debouncedEndstopState = rawState;
      lastRawEndstopState = rawState;
      lastEndstopChangeTime = millis();

      // Configure endstop ISR globals and attach hardware interrupt
      g_endstopPin = (uint8_t)endstopPin;
      g_endstopActiveLow = endstopActiveLow;
      g_rpwmChannel = (uint8_t)pwmChannelR;
      g_lpwmChannel = (uint8_t)pwmChannelL;
      g_renPin = (uint8_t)renPin;
      g_lenPin = (uint8_t)lenPin;
      g_endstopMotorArmed = false;
      g_endstopISRFired = false;
      attachInterrupt(digitalPinToInterrupt(endstopPin), endstop_isr_handler,
                      endstopActiveLow ? FALLING : RISING);
    }

    // Hall inputs (no internal pullup — HE sensor board provides external pullups)
    pinMode(hallAPin, INPUT);
    pinMode(hallBPin, INPUT);

    // Initialize ISR quadrature state from current AB levels
    g_hallLastAB = ((uint8_t)digitalRead(hallAPin) << 1) | (uint8_t)digitalRead(hallBPin);
    g_hallInit = true;

    // Configure ISR globals (must be set before attachInterrupt)
    g_hallAPin = (uint8_t)hallAPin;
    g_hallBPin = (uint8_t)hallBPin;

    // Attach interrupts using a FREE FUNCTION ISR (fixes linker errors)
    attachInterrupt(digitalPinToInterrupt(hallAPin), hall_isr_capture, CHANGE);
    attachInterrupt(digitalPinToInterrupt(hallBPin), hall_isr_capture, CHANGE);

#if defined(ARDUINO_ARCH_ESP32)
    // PWM LEDC setup — two channels for BTS7960 RPWM/LPWM
    ledcSetup(pwmChannelR, pwmFrequency, pwmResolution);
    ledcAttachPin(rpwmPin, pwmChannelR);
    ledcSetup(pwmChannelL, pwmFrequency, pwmResolution);
    ledcAttachPin(lpwmPin, pwmChannelL);
#endif

    // Ensure motor is stopped on startup — disable both half-bridges
    coastMotor();

    // BTS7960 IS pin ADC setup
    if (currentSenseEnabled) {
      pinMode(rIsPin, INPUT);
      pinMode(lIsPin, INPUT);
#if defined(ARDUINO_ARCH_ESP32)
      analogSetAttenuation(ADC_11db);  // 0–3.3V range
      analogReadResolution(12);        // 12-bit (0–4095)
#endif
    }

    // ---------------------------
    // Boot-time endstop check
    // ---------------------------
    if (endstopEnabled) {
      if (isEndstopTriggered()) {
        // Endstop is triggered at boot — motor is at home (bottom)
        isHomed = true;
        currentPositionMm = 0.0f;
        positionTicks = 0;
        setDirectionUp();  // Next movement should go UP
      } else {
        // Endstop NOT triggered — position is unknown.
        // First button press will auto-home (slow descent until stall, back off, set home).
        // Users can also manually jog down and press "Set Bottom".
        // Motor will NOT move automatically (safety: prevent pinch on power-up).
        isHomed = false;
        currentPositionMm = 0.0f;
      }
    }
    // LED state is deferred to first loop() iteration (bootLedApplied flag)
    // so that WLED's boot preset has time to load the user's saved color/brightness.

    initDone = true;
  }

  void loop() {
    if (!enabled || !initDone) return;

    // Deferred boot LED: apply on first loop iteration after WLED's boot preset
    // has loaded the user's saved color/brightness settings.
    if (!bootLedApplied) {
      bootLedApplied = true;
      if (isHomed && currentPositionMm <= ledOffDistanceMm) {
        // Shelf is at bottom — save the user's boot brightness, then turn off
        if (bri > 0) ledSavedBri = bri;
        bri = 0;
        stateUpdated(CALL_MODE_DIRECT_CHANGE);
      }
      // If shelf is up or not homed, leave WLED's loaded state alone
    }

    // Motor control must run every iteration regardless of strip updates.
    // GPIO reads, PWM writes, and button detection are safe during DMA.

    // Process hall updates frequently
    processHallIfPending();

    // Update LED state based on current position (reacts to movement)
    updateLedState();

    // Update debounced endstop state every iteration
    updateEndstopDebounce();

    // ---------------------------
    // Endstop monitoring (every loop iteration for safety)
    // ---------------------------
    if (endstopEnabled) {
      // Check if the hardware ISR already killed the motor (fires within microseconds
      // of endstop contact, independent of loop() timing).  We just do cleanup here.
      if (g_endstopISRFired) {
        g_endstopISRFired = false;
        // Motor power is already cut by ISR — just sync the state machine
        lastStopReason = STOP_ENDSTOP;
        lastActiveMovingDown = isMovingDown();
        motorState = IDLE;
        kickActive = false;
        spikeSampleCount = 0;
        currentPwm = 0;
        manualJogActive = false;
        homeSeekActive = false;
        autoHomingActive = false;
        autoHomingPhaseBackoff = false;
        isHomed = true;
        currentPositionMm = 0.0f;
        positionTicks = 0;
        setDirectionUp();
        updateLedState();
        publishHomeAssistantSensor();
        serializeConfigToFS();
        return;
      }

      bool endstopHit = isEndstopTriggered();

      // Polled fallback: if endstop triggers while motor is moving DOWN — IMMEDIATE stop
      // (covers edge cases where ISR might not be armed, e.g. direction change mid-run)
      if (endstopHit && motorState != IDLE && isMovingDown()) {
        immediateStop(STOP_ENDSTOP);
        isHomed = true;
        currentPositionMm = 0.0f;
        positionTicks = 0;
        setDirectionUp();  // Next movement goes UP
        return;            // Skip rest of loop this iteration
      }

      // If endstop triggered while idle and not yet homed (e.g., manually pushed)
      if (endstopHit && motorState == IDLE && !isHomed) {
        isHomed = true;
        currentPositionMm = 0.0f;
        positionTicks = 0;
        setDirectionUp();
      }
    }

    // ---------------------------
    // Travel limit checks while moving UP
    // ---------------------------
    // Manual jog ignores user maxTravelDistance, but still enforces firmware hard limit.
    // Normal movement enforces user maxTravelDistance (already clamped to firmware limit).
    if (endstopEnabled && isHomed &&
        (motorState == STARTING || motorState == RUNNING) &&
        isMovingUp()) {
      if (manualJogActive) {
        if (currentPositionMm >= FIRMWARE_MAX_TRAVEL_MM) {
          immediateStop(STOP_TRAVEL_LIMIT);
          return;
        }
      } else {
        if (currentPositionMm >= maxTravelDistance) {
          beginStop(STOP_TRAVEL_LIMIT);
          return;
        }
      }
    }

    // Button toggles start/stop
    if (buttonPressed()) {
      handleToggleAction();
    }

    // State machine
    if (motorState == STARTING) {
      if (!manualJogActive && !autoHomingActive && millis() - runStartTime >= safetyMaxRunMs) {
        beginStop(STOP_TIMEOUT);
      } else {
        const unsigned long runElapsed = millis() - runStartTime;
        // During ramp-up, allow some time to avoid false trips before the motor gains momentum.
        // Hard-stop checks remain fully active in RUNNING state.
        if (runElapsed >= stallStartGraceMs && pollCurrentAndCheckSpike()) {
          if (autoHomingActive && !autoHomingPhaseBackoff) {
            beginAutoHomingBackoff();
          } else if (autoHomingActive && autoHomingPhaseBackoff) {
            completeAutoHoming();
          } else {
            immediateStop(STOP_SPIKE, !manualJogActive);
          }
          return;
        }
        updateStartSequence();
      }
      return;
    }

    if (motorState == RUNNING) {
      const unsigned long elapsed = millis() - runStartTime;

      // Safety timeout (skipped during auto-homing — stall detection is the stop mechanism)
      if (!manualJogActive && !autoHomingActive && elapsed >= safetyMaxRunMs) {
        beginStop(STOP_TIMEOUT);
        return;
      }

      // Target distance reached => smooth auto-stop
      if (!manualJogActive && !autoHomingActive && hasReachedTargetDistance()) {
        beginStop(STOP_TARGET_REACHED);
        return;
      }

      // Auto-homing backoff: stop after backing up homingBackoffMm from stall point
      if (autoHomingActive && autoHomingPhaseBackoff) {
        if (getDistanceTraveled() >= homingBackoffMm) {
          completeAutoHoming();
          return;
        }
      }

      // Hard floor: bottom is a physical stop — immediate power cut at 0mm.
      if (isHomed && !homeSeekActive && !manualJogActive &&
          isMovingDown() && currentPositionMm <= 0.0f) {
        immediateStop(STOP_TARGET_REACHED, true);
        currentPositionMm = 0.0f;
        return;
      }

      // Bottom approach: proportionally reduce speed as the motor nears 0mm.
      // Motor keeps moving (no state change) but arrives at low speed so the
      // hard floor stop is gentle.  Returns early to prevent the default
      // applyPwm(currentPwm) at the end of RUNNING from overwriting.
      if (isHomed && !homeSeekActive && !manualJogActive &&
          isMovingDown() && bottomApproachMm > 0.0f &&
          currentPositionMm <= bottomApproachMm) {
        float fraction = currentPositionMm / bottomApproachMm; // 1.0 at edge, 0.0 at bottom
        int approachPwm = pwmMin + (int)((float)(pwmMax - pwmMin) * fraction);
        if (approachPwm < pwmMin) approachPwm = pwmMin;
        setDirectionPins(motorDirection);
        applyPwm(approachPwm);
        return;
      }

      // Encoder stall detection (Safety Layer 2): no hall tick in stallTimeoutMs
      if (stallDetectionEnabled) {
        bool stalled = false;
        if (!hallSeenThisRun) {
          if (elapsed > (accelTimeMs + stallStartGraceMs)) stalled = true;
        } else if (millis() - lastHallTickTime > stallTimeoutMs) {
          stalled = true;
        }
        if (stalled) {
          if (autoHomingActive && !autoHomingPhaseBackoff) {
            // Stall during homing seek-down = found bottom, start backoff
            beginAutoHomingBackoff();
            return;
          }
          if (autoHomingActive && autoHomingPhaseBackoff) {
            // Stall during backoff — set home at current position
            completeAutoHoming();
            return;
          }
          immediateStop(STOP_STALL, !manualJogActive);
          return;
        }
      }

      // Current spike => immediate hard stop for minimum fault latency.
      if (pollCurrentAndCheckSpike()) {
        if (autoHomingActive && !autoHomingPhaseBackoff) {
          beginAutoHomingBackoff();
          return;
        }
        if (autoHomingActive && autoHomingPhaseBackoff) {
          completeAutoHoming();
          return;
        }
        immediateStop(STOP_SPIKE, !manualJogActive);
        return;
      }

      // Maintain direction and PWM (DO NOT overwrite with pwmMax here,
      // otherwise it cancels acceleration behavior)
      setDirectionPins(motorDirection);
      applyPwm(currentPwm); // typically pwmMax after ramp completes
      return;
    }

    if (motorState == STOPPING) {
      // Hard floor applies during decel too — if the motor is decelerating
      // toward 0mm, cut power the moment it arrives.  Without this, the decel
      // ramp lets the motor coast past 0mm causing cumulative drift.
      if (isHomed && !homeSeekActive && !manualJogActive &&
          isMovingDown() && currentPositionMm <= 0.0f) {
        immediateStop(STOP_TARGET_REACHED, true);
        currentPositionMm = 0.0f;
        return;
      }
      updateStopSequence();
      return;
    }

    // IDLE: nothing
  }

  void addToJsonInfo(JsonObject& root) {
    JsonObject user = root["u"];
    if (user.isNull()) user = root.createNestedObject("u");

    JsonArray motorStatus = user.createNestedArray("Motor");
    const char* st =
      (motorState == RUNNING)  ? "Running" :
      (motorState == STARTING) ? "Starting" :
      (motorState == STOPPING) ? "Stopping" :
                                 "Idle";
    motorStatus.add(st);
    motorStatus.add(motorDirection ? "Forward" : "Reverse");

    // Homing / position status
    JsonArray homeStatus = user.createNestedArray("Bottom Set");
    if (autoHomingActive) {
      homeStatus.add(autoHomingPhaseBackoff ? "Auto-homing (backing off)" : "Auto-homing (seeking bottom)");
    } else {
      homeStatus.add(isHomed ? "Yes" : "NO - press button to auto-home");
    }

    if (endstopEnabled) {
      JsonArray endstopStatus = user.createNestedArray("Endstop");
      endstopStatus.add(isEndstopTriggered() ? "TRIGGERED" : "Open");
    }

    if (isHomed) {
      JsonArray absPos = user.createNestedArray("Position (mm)");
      absPos.add(currentPositionMm);
      absPos.add(" / ");
      absPos.add(maxTravelDistance);
    }

    // Show detected direction from hall sensors (actual spin direction)
    JsonArray hallDir = user.createNestedArray("Hall Direction");
    const char* detDir =
      (detectedDirection > 0) ? "Forward" :
      (detectedDirection < 0) ? "Reverse" :
                                "Unknown";
    hallDir.add(detDir);
    hallDir.add(directionConfidence); // confidence level

    JsonArray pos = user.createNestedArray("Position (ticks)");
    pos.add((int32_t)positionTicks);

    // Show distance traveled in current run
    JsonArray dist = user.createNestedArray("Distance");
    dist.add(getDistanceTraveled());
    if (targetDistanceEnabled && targetDistance > 0) {
      dist.add(" / ");
      dist.add(targetDistance);
    }

    JsonArray cur = user.createNestedArray("Current (mA)");
    cur.add(lastCurrentmA);

    JsonArray pwm = user.createNestedArray("PWM");
    pwm.add(currentPwm);

    // Show last stop reason
    if (motorState == IDLE && lastStopReason != STOP_USER) {
      JsonArray stopReason = user.createNestedArray("Last Stop");
      const char* reason =
        (lastStopReason == STOP_SPIKE)          ? "Current Spike" :
        (lastStopReason == STOP_TIMEOUT)        ? "Timeout" :
        (lastStopReason == STOP_TARGET_REACHED) ? "Target Reached" :
        (lastStopReason == STOP_ENDSTOP)        ? "Endstop Hit" :
        (lastStopReason == STOP_TRAVEL_LIMIT)   ? "Travel Limit" :
        (lastStopReason == STOP_STALL)          ? "Stall Detected" :
                                                  "User";
      stopReason.add(reason);
    }

    if (!currentSenseEnabled) {
      JsonArray warn = user.createNestedArray("Current Sense");
      warn.add("DISABLED");
    }

    if (renPin == 12) {
      JsonArray warn = user.createNestedArray("Pin Warning");
      warn.add("renPin=GPIO12 can break boot/OTA; use non-strapping pin (e.g. 25)");
    }

    // LED control status
    if (ledControlEnabled) {
      JsonArray ledCtrl = user.createNestedArray("LED Control");
      ledCtrl.add(ledInvertDirection ? "Inverted" : "Normal");
    }

    // Add control buttons
    JsonArray btn = user.createNestedArray(F("Motor Control"));
    String buttonHtml = F("<button class=\"btn btn-xs\" onclick=\"requestJson({motorController:{panelToggle:true}});\">");
    if (motorState == IDLE) {
      buttonHtml += F("<i class=\"icons off\">&#xe08f;</i> Start");
    } else {
      buttonHtml += F("<i class=\"icons on\">&#xe08f;</i> Stop");
    }
    buttonHtml += F("</button>");

    // Set Bottom: mark current position as home (0mm).
    buttonHtml += F(" <button class=\"btn btn-xs\" onclick=\"requestJson({motorController:{setHome:true}});\">");
    buttonHtml += F("<i class=\"icons\">&#8962;</i> Set Bottom");
    buttonHtml += F("</button>");

    // Fine control: press-and-hold jog buttons.
    // Press starts movement; release/cancel/leave sends stop.
    buttonHtml += F(" <button class=\"btn btn-xs\" onpointerdown=\"requestJson({motorController:{jogUp:true}});\" ");
    buttonHtml += F("onpointerup=\"requestJson({motorController:{jogStop:true}});\" ");
    buttonHtml += F("onpointercancel=\"requestJson({motorController:{jogStop:true}});\" ");
    buttonHtml += F("onmouseleave=\"requestJson({motorController:{jogStop:true}});\" ");
    buttonHtml += F("ontouchend=\"requestJson({motorController:{jogStop:true}});\">");
    buttonHtml += F("&#8593; Up");
    buttonHtml += F("</button>");

    buttonHtml += F(" <button class=\"btn btn-xs\" onpointerdown=\"requestJson({motorController:{jogDown:true}});\" ");
    buttonHtml += F("onpointerup=\"requestJson({motorController:{jogStop:true}});\" ");
    buttonHtml += F("onpointercancel=\"requestJson({motorController:{jogStop:true}});\" ");
    buttonHtml += F("onmouseleave=\"requestJson({motorController:{jogStop:true}});\" ");
    buttonHtml += F("ontouchend=\"requestJson({motorController:{jogStop:true}});\">");
    buttonHtml += F("&#8595; Down");
    buttonHtml += F("</button>");

    btn.add(buttonHtml);
    if (!isHomed) {
      btn.add(F(" (Set bottom first)"));
    } else {
      btn.add(motorDirection ? F(" (Next: Fwd)") : F(" (Next: Rev)"));
    }
  }

  void addToJsonState(JsonObject& root) {
    JsonObject usermod = root.createNestedObject("motorController");
    usermod["running"] = (motorState == RUNNING || motorState == STARTING || motorState == STOPPING);
    usermod["state"] = (motorState == RUNNING) ? "running" :
                       (motorState == STARTING) ? "starting" :
                       (motorState == STOPPING) ? "stopping" : "idle";
    usermod["direction"] = motorDirection ? "forward" : "reverse";

    // Detected direction from hall sensors (actual motor spin)
    usermod["hallDirection"] = (detectedDirection > 0) ? "forward" :
                               (detectedDirection < 0) ? "reverse" : "unknown";
    usermod["hallConfidence"] = directionConfidence;

    usermod["posTicks"] = (int32_t)positionTicks;
    usermod["ticksPerMm"] = ticksPerMm;
    usermod["distance"] = getDistanceTraveled();
    usermod["targetDistance"] = targetDistance;
    usermod["targetEnabled"] = targetDistanceEnabled;
    usermod["current_mA"] = lastCurrentmA;
    usermod["pwm"] = currentPwm;

    // Last stop reason
    usermod["lastStopReason"] = (lastStopReason == STOP_SPIKE) ? "spike" :
                                (lastStopReason == STOP_TIMEOUT) ? "timeout" :
                                (lastStopReason == STOP_TARGET_REACHED) ? "target" :
                                (lastStopReason == STOP_ENDSTOP) ? "endstop" :
                                (lastStopReason == STOP_TRAVEL_LIMIT) ? "travel_limit" :
                                (lastStopReason == STOP_STALL) ? "stall" : "user";

    // Endstop / homing state
    usermod["isHomed"] = isHomed;
    usermod["autoHoming"] = autoHomingActive;
    usermod["autoHomingPhase"] = autoHomingActive ? (autoHomingPhaseBackoff ? "backoff" : "seeking") : "none";
    usermod["endstopTriggered"] = endstopEnabled ? isEndstopTriggered() : false;
    usermod["currentPositionMm"] = currentPositionMm;
    usermod["maxTravelDistance"] = maxTravelDistance;
    usermod["firmwareMaxTravel"] = FIRMWARE_MAX_TRAVEL_MM;

    // LED control state
    usermod["ledControlEnabled"] = ledControlEnabled;
    usermod["ledInvertDirection"] = ledInvertDirection;
  }

  void readFromJsonState(JsonObject& root) {
    JsonObject usermod = root["motorController"];
    if (usermod.isNull()) return;

    // Allow setting target distance via API (clamped to firmware limit)
    if (!usermod["targetDistance"].isNull()) {
      targetDistance = usermod["targetDistance"].as<float>();
      if (targetDistance > maxTravelDistance) targetDistance = maxTravelDistance;
      if (targetDistance > FIRMWARE_MAX_TRAVEL_MM) targetDistance = FIRMWARE_MAX_TRAVEL_MM;
    }
    if (!usermod["targetEnabled"].isNull()) {
      targetDistanceEnabled = usermod["targetEnabled"].as<bool>();
    }

    // Allow setting max travel distance via API (clamped to firmware limit)
    if (!usermod["maxTravelDistance"].isNull()) {
      maxTravelDistance = usermod["maxTravelDistance"].as<float>();
      if (maxTravelDistance > FIRMWARE_MAX_TRAVEL_MM) maxTravelDistance = FIRMWARE_MAX_TRAVEL_MM;
      if (maxTravelDistance < 0) maxTravelDistance = 0;
      // Re-clamp target distance if needed
      if (targetDistance > maxTravelDistance) targetDistance = maxTravelDistance;
    }

    // Allow setting ticks per mm via API
    if (!usermod["ticksPerMm"].isNull()) {
      ticksPerMm = usermod["ticksPerMm"].as<float>();
      if (ticksPerMm <= 0) ticksPerMm = 1.0f;
    }

    // Toggle (simulates button press)
    if (usermod["toggle"].as<bool>()) {
      handleToggleAction();
    }

    // Info panel main button
    if (usermod["panelToggle"].as<bool>()) {
      handleToggleAction();
    }

    // Explicit start/stop commands
    if (usermod["start"].as<bool>() && motorState == IDLE) {
      homeSeekActive = false;
      beginStart();
    }

    if (usermod["stop"].as<bool>() && motorState != IDLE) {
      if (manualJogActive) immediateStop(STOP_USER);
      else beginStop(STOP_USER);
    }

    // Manual jog commands from Info UI (press-and-hold controls)
    if (usermod["jogUp"].as<bool>()) {
      startManualJog(true);
    }
    if (usermod["jogDown"].as<bool>()) {
      startManualJog(false);
    }
    if (usermod["jogStop"].as<bool>() && motorState != IDLE) {
      if (manualJogActive) immediateStop(STOP_USER);
      else beginStop(STOP_USER);
    }

    // Direction-aware open/close commands
    // Uses ledInvertDirection to determine which physical direction is "open"
    if (usermod["open"].as<bool>() && motorState == IDLE) {
      homeSeekActive = false;
      motorDirection = !ledInvertDirection;  // Set to opening direction
      beginStart();
    }

    if (usermod["close"].as<bool>() && motorState == IDLE) {
      homeSeekActive = false;
      motorDirection = ledInvertDirection;  // Set to closing direction
      beginStart();
    }

    // Set Home: mark current position as bottom (0mm).
    // User jogs the shelf to desired position, then presses "Set Bottom".
    if (usermod["setHome"].as<bool>() && motorState == IDLE) {
      isHomed = true;
      currentPositionMm = 0.0f;
      positionTicks = 0;
      setDirectionUp();  // Next movement goes UP (away from new home)
      publishHomeAssistantSensor();
    }

    // Auto-home: seek bottom by stalling, then back off and set home
    if (usermod["autoHome"].as<bool>() && motorState == IDLE && !isHomed) {
      autoHomingActive = true;
      autoHomingPhaseBackoff = false;
      homeSeekActive = true;
      setDirectionDown();
      beginStart();
    }

    // Legacy home command kept for API compatibility — also sets home at current position
    if (usermod["home"].as<bool>() && motorState == IDLE) {
      isHomed = true;
      currentPositionMm = 0.0f;
      positionTicks = 0;
      setDirectionUp();
      publishHomeAssistantSensor();
    }

    if (usermod["resetPos"].as<bool>()) {
      positionTicks = 0;
      runStartTicks = 0;
    }

    // Allow toggling LED control via API
    if (!usermod["ledControlEnabled"].isNull()) {
      ledControlEnabled = usermod["ledControlEnabled"].as<bool>();
    }
    if (!usermod["ledInvertDirection"].isNull()) {
      ledInvertDirection = usermod["ledInvertDirection"].as<bool>();
    }
  }

  void addToConfig(JsonObject& root) {
    JsonObject top = root.createNestedObject("MotorController");
    top["enabled"] = enabled;

    top["rpwmPin"] = rpwmPin;
    top["lpwmPin"] = lpwmPin;
    top["renPin"] = renPin;
    top["lenPin"] = lenPin;
    top["buttonPin"] = buttonPin;

    top["hallAPin"] = hallAPin;
    top["hallBPin"] = hallBPin;

    // Endstop configuration
    top["endstopPin"] = endstopPin;
    top["endstopActiveLow"] = endstopActiveLow;
    top["endstopEnabled"] = endstopEnabled;
    top["endstopDebounceMs"] = endstopDebounceMs;
    top["bottomApproachMm"] = bottomApproachMm;

    // Auto-homing settings
    top["homingPwmMax"] = homingPwmMax;
    top["homingBackoffMm"] = homingBackoffMm;

    // Direction mapping
    top["downIsForward"] = downIsForward;
    top["invertMotorDirection"] = invertMotorDirection;

    // Stall detection
    top["stallDetectionEnabled"] = stallDetectionEnabled;
    top["stallTimeoutMs"] = stallTimeoutMs;
    top["stallStartGraceMs"] = stallStartGraceMs;

    // Travel limit
    top["maxTravelDistance"] = maxTravelDistance;

    // Distance configuration
    top["ticksPerMm"] = ticksPerMm;
    top["targetDistance"] = targetDistance;
    top["targetDistanceEnabled"] = targetDistanceEnabled;

    top["accelTime"] = accelTimeMs;
    top["decelTime"] = decelTimeMs;
    top["safetyMaxRunMs"] = safetyMaxRunMs;

    top["pwmMin"] = pwmMin;
    top["pwmMax"] = pwmMax;
    top["pwmFrequency"] = pwmFrequency;

    top["kickstartEnabled"] = kickstartEnabled;
    top["kickstartPwm"] = kickstartPwm;
    top["kickstartMs"] = kickstartMs;

    // BTS7960 current sense
    top["rIsPin"] = rIsPin;
    top["lIsPin"] = lIsPin;
    top["isResistorOhms"] = isResistorOhms;
    top["isRatio"] = isRatio;
    top["currentSenseEnabled"] = currentSenseEnabled;
    top["adcSampleCount"] = adcSampleCount;

    top["currentPollMs"] = currentPollMs;
    top["currentSpikeThresholdmA"] = currentSpikeThresholdmA;
    top["spikeSamplesRequired"] = spikeSamplesRequired;

    // LED control settings
    top["ledControlEnabled"] = ledControlEnabled;
    top["ledInvertDirection"] = ledInvertDirection;
    top["ledSavedBri"] = ledSavedBri;
    top["ledOffDistanceMm"] = ledOffDistanceMm;

    // Persist position across reboots (motor can't move while off)
    top["savedPositionMm"] = currentPositionMm;
    top["savedIsHomed"] = isHomed;
  }

  bool readFromConfig(JsonObject& root) {
    // Defaults
    enabled = true;
    rpwmPin = 27;
    lpwmPin = 14;
    renPin = 25;
    lenPin = 13;
    buttonPin = 33;

    hallAPin = 32;
    hallBPin = 35;

    // Endstop defaults
    endstopPin = 22;
    endstopActiveLow = true;
    endstopEnabled = true;
    endstopDebounceMs = 20;
    bottomApproachMm = 50.0f;

    // Auto-homing defaults
    homingPwmMax = 100;
    homingBackoffMm = 2.0f;

    // Direction mapping defaults
    downIsForward = false;
    invertMotorDirection = false;

    // Stall detection defaults
    stallDetectionEnabled = true;
    stallTimeoutMs = 150;
    stallStartGraceMs = 100;

    // Travel limit default
    maxTravelDistance = FIRMWARE_MAX_TRAVEL_MM;

    // Distance defaults
    targetDistance = FIRMWARE_MAX_TRAVEL_MM;
    targetDistanceEnabled = true;
    ticksPerMm = 1.0f;

    accelTimeMs = 1500;
    decelTimeMs = 800;
    safetyMaxRunMs = 20000;

    pwmMin = 0;
    pwmMax = 255;
    pwmFrequency = 1000;

    kickstartEnabled = false;
    kickstartPwm = 255;
    kickstartMs = 120;

    // BTS7960 IS pin defaults
    rIsPin = 39;
    lIsPin = 36;
    isResistorOhms = 4700.0f;
    isRatio = 8500.0f;
    currentSenseEnabled = true;
    adcSampleCount = 4;

    currentPollMs = 10;
    currentSpikeThresholdmA = 3500.0f;
    spikeSamplesRequired = 1;

    // LED control defaults
    ledControlEnabled = true;
    ledInvertDirection = false;
    ledSavedBri = 128;
    ledOffDistanceMm = 10.0f;

    JsonObject top = root["MotorController"];
    if (top.isNull()) return false;

    bool ok = true;
    ok &= getJsonValue(top["enabled"], enabled);

    ok &= getJsonValue(top["rpwmPin"], rpwmPin);
    ok &= getJsonValue(top["lpwmPin"], lpwmPin);
    ok &= getJsonValue(top["renPin"], renPin);
    ok &= getJsonValue(top["lenPin"], lenPin);
    ok &= getJsonValue(top["buttonPin"], buttonPin);

    ok &= getJsonValue(top["hallAPin"], hallAPin);
    ok &= getJsonValue(top["hallBPin"], hallBPin);
    if (hallAPin < 0 || hallAPin > 39) hallAPin = 32;
    if (hallBPin < 0 || hallBPin > 39) hallBPin = 35;

    // Endstop configuration
    ok &= getJsonValue(top["endstopPin"], endstopPin);
    ok &= getJsonValue(top["endstopActiveLow"], endstopActiveLow);
    ok &= getJsonValue(top["endstopEnabled"], endstopEnabled);
    ok &= getJsonValue(top["endstopDebounceMs"], endstopDebounceMs);
    ok &= getJsonValue(top["bottomApproachMm"], bottomApproachMm);

    // Auto-homing settings
    ok &= getJsonValue(top["homingPwmMax"], homingPwmMax);
    ok &= getJsonValue(top["homingBackoffMm"], homingBackoffMm);
    if (homingPwmMax < 30) homingPwmMax = 30;
    if (homingPwmMax > 255) homingPwmMax = 255;
    if (homingBackoffMm < 0) homingBackoffMm = 0;
    if (homingBackoffMm > 50) homingBackoffMm = 50;

    // Direction mapping
    ok &= getJsonValue(top["downIsForward"], downIsForward);
    ok &= getJsonValue(top["invertMotorDirection"], invertMotorDirection);

    // Stall detection
    ok &= getJsonValue(top["stallDetectionEnabled"], stallDetectionEnabled);
    ok &= getJsonValue(top["stallTimeoutMs"], stallTimeoutMs);
    ok &= getJsonValue(top["stallStartGraceMs"], stallStartGraceMs);

    // Travel limit (clamped to firmware max)
    ok &= getJsonValue(top["maxTravelDistance"], maxTravelDistance);
    if (maxTravelDistance > FIRMWARE_MAX_TRAVEL_MM) maxTravelDistance = FIRMWARE_MAX_TRAVEL_MM;
    if (maxTravelDistance < 0) maxTravelDistance = 0;

    // Distance configuration
    ok &= getJsonValue(top["targetDistance"], targetDistance);
    ok &= getJsonValue(top["targetDistanceEnabled"], targetDistanceEnabled);
    ok &= getJsonValue(top["ticksPerMm"], ticksPerMm);

    // Validate ticksPerMm
    if (ticksPerMm <= 0) ticksPerMm = 1.0f;

    // Clamp target distance to travel limit
    if (targetDistance > maxTravelDistance) targetDistance = maxTravelDistance;
    if (targetDistance > FIRMWARE_MAX_TRAVEL_MM) targetDistance = FIRMWARE_MAX_TRAVEL_MM;

    ok &= getJsonValue(top["accelTime"], accelTimeMs);
    ok &= getJsonValue(top["decelTime"], decelTimeMs);
    ok &= getJsonValue(top["safetyMaxRunMs"], safetyMaxRunMs);

    ok &= getJsonValue(top["pwmMin"], pwmMin);
    ok &= getJsonValue(top["pwmMax"], pwmMax);
    ok &= getJsonValue(top["pwmFrequency"], pwmFrequency);

    ok &= getJsonValue(top["kickstartEnabled"], kickstartEnabled);
    ok &= getJsonValue(top["kickstartPwm"], kickstartPwm);
    ok &= getJsonValue(top["kickstartMs"], kickstartMs);

    // BTS7960 IS pin config
    ok &= getJsonValue(top["rIsPin"], rIsPin);
    ok &= getJsonValue(top["lIsPin"], lIsPin);
    ok &= getJsonValue(top["isResistorOhms"], isResistorOhms);
    ok &= getJsonValue(top["isRatio"], isRatio);
    ok &= getJsonValue(top["currentSenseEnabled"], currentSenseEnabled);
    ok &= getJsonValue(top["adcSampleCount"], adcSampleCount);
    if (adcSampleCount < 1) adcSampleCount = 1;
    if (adcSampleCount > 16) adcSampleCount = 16;

    // Validate IS parameters
    if (isResistorOhms <= 0) isResistorOhms = 4700.0f;
    if (isRatio <= 0) isRatio = 8500.0f;

    ok &= getJsonValue(top["currentPollMs"], currentPollMs);
    ok &= getJsonValue(top["currentSpikeThresholdmA"], currentSpikeThresholdmA);
    ok &= getJsonValue(top["spikeSamplesRequired"], spikeSamplesRequired);
    if (currentPollMs < 1) currentPollMs = 1;
    if (spikeSamplesRequired < 1) spikeSamplesRequired = 1;

    // LED control settings
    ok &= getJsonValue(top["ledControlEnabled"], ledControlEnabled);
    ok &= getJsonValue(top["ledInvertDirection"], ledInvertDirection);
    ok &= getJsonValue(top["ledSavedBri"], ledSavedBri);
    ok &= getJsonValue(top["ledOffDistanceMm"], ledOffDistanceMm);
    if (ledSavedBri < 1) ledSavedBri = 128;
    if (ledOffDistanceMm < 0) ledOffDistanceMm = 0;

    // Restore saved position across reboots (motor can't move while off)
    float savedPos = 0.0f;
    bool savedHomed = false;
    if (getJsonValue(top["savedPositionMm"], savedPos) &&
        getJsonValue(top["savedIsHomed"], savedHomed) && savedHomed) {
      if (savedPos >= 0.0f && savedPos <= FIRMWARE_MAX_TRAVEL_MM) {
        currentPositionMm = savedPos;
        isHomed = true;
      }
    }

    // Update ISR globals if pins changed via config (NOTE: interrupts already attached)
    g_hallAPin = (uint8_t)hallAPin;
    g_hallBPin = (uint8_t)hallBPin;

    // If PWM frequency changed via config, re-init both LEDC channels
#if defined(ARDUINO_ARCH_ESP32)
    if (initDone) {
      ledcDetachPin(rpwmPin);
      ledcDetachPin(lpwmPin);
      ledcSetup(pwmChannelR, pwmFrequency, pwmResolution);
      ledcAttachPin(rpwmPin, pwmChannelR);
      ledcSetup(pwmChannelL, pwmFrequency, pwmResolution);
      ledcAttachPin(lpwmPin, pwmChannelL);
      applyPwm(currentPwm);
    }
#endif

    return ok;
  }

  void appendConfigData() override {
    // Helper macro for cleaner code
    #define MCINFO(field, text) oappend(F("addInfo('MotorController:" field "',1,'" text "');"))

    // Main enable
    MCINFO("enabled", "<i>Enable motor controller usermod</i>");

    // Pin configuration section (BTS7960)
    MCINFO("rpwmPin", "<i>BTS7960 RPWM — PWM for extend direction</i>");
    MCINFO("lpwmPin", "<i>BTS7960 LPWM — PWM for retract direction</i>");
    MCINFO("renPin", "<i>BTS7960 R_EN — enable extend half-bridge</i>");
    MCINFO("lenPin", "<i>BTS7960 L_EN — enable retract half-bridge</i>");
    MCINFO("buttonPin", "<i>Momentary button input (to GND)</i>");
    MCINFO("hallAPin", "<i>Quadrature encoder channel A</i>");
    MCINFO("hallBPin", "<i>Quadrature encoder channel B</i>");

    // Endstop settings
    MCINFO("endstopPin", "<i>Bottom endstop switch input (GPIO34-39 need external pull-up)</i>");
    MCINFO("endstopActiveLow", "<i>true = LOW when triggered (NO switch + pull-up)</i>");
    MCINFO("endstopEnabled", "<i>Enable bottom endstop for homing &amp; safety</i>");
    MCINFO("endstopDebounceMs", "ms <i>Endstop switch debounce time (10-50ms for mechanical)</i>");
    MCINFO("bottomApproachMm", "mm <i>Start decelerating this far above bottom (0 = hard stop only)</i>");

    // Auto-homing
    MCINFO("homingPwmMax", "<i>PWM speed for auto-homing seek (30-255, lower = slower/safer)</i>");
    MCINFO("homingBackoffMm", "mm <i>Distance to back off after finding bottom (0-50)</i>");

    // Direction & travel
    MCINFO("downIsForward", "<i>false = reverse is DOWN (toward endstop)</i>");
    MCINFO("invertMotorDirection", "<i>Swap RPWM/LPWM logic if motor runs backwards</i>");

    // Stall detection
    MCINFO("stallDetectionEnabled", "<i>Stop motor if encoder reports no movement</i>");
    MCINFO("stallTimeoutMs", "ms <i>No encoder pulse for this long = stalled (100-300)</i>");
    MCINFO("stallStartGraceMs", "ms <i>Grace period after start before stall checks begin</i>");
    MCINFO("maxTravelDistance", "mm <i>Max UP travel from bottom (firmware limit: 762mm / 30&quot;)</i>");

    // Distance/position settings
    MCINFO("ticksPerMm", "<i>Hall ticks per millimeter for distance/position</i>");
    MCINFO("targetDistance", "<i>Auto-stop distance, clamped to max travel (0 = disabled)</i>");
    MCINFO("targetDistanceEnabled", "<i>Enable auto-stop at target distance</i>");

    // Motion timing
    MCINFO("accelTime", "ms <i>S-curve ramp up duration</i>");
    MCINFO("decelTime", "ms <i>S-curve ramp down duration</i>");
    MCINFO("safetyMaxRunMs", "ms <i>Maximum run time before auto-stop</i>");

    // PWM settings
    MCINFO("pwmMin", "<i>Starting PWM (0-255)</i>");
    MCINFO("pwmMax", "<i>Full speed PWM (0-255)</i>");
    MCINFO("pwmFrequency", "Hz <i>Motor driver PWM frequency</i>");

    // Kickstart settings
    MCINFO("kickstartEnabled", "<i>Brief full-power pulse to overcome friction</i>");
    MCINFO("kickstartPwm", "<i>Kickstart pulse PWM (0-255)</i>");
    MCINFO("kickstartMs", "ms <i>Kickstart pulse duration</i>");

    // BTS7960 current sensing (IS pins)
    MCINFO("rIsPin", "<i>BTS7960 R_IS analog input (ADC1 pin)</i>");
    MCINFO("lIsPin", "<i>BTS7960 L_IS analog input (ADC1 pin)</i>");
    MCINFO("isResistorOhms", "&Omega; <i>IS-to-GND sense resistor (e.g., 4700 for 4.7k&Omega;)</i>");
    MCINFO("isRatio", "<i>Current sense ratio (typ ~8500, calibrate per batch)</i>");
    MCINFO("currentSenseEnabled", "<i>Enable BTS7960 IS pin current sensing</i>");
    MCINFO("adcSampleCount", "<i>ADC samples to average per reading (1-16, reduces noise)</i>");
    MCINFO("currentPollMs", "ms <i>Current sense polling interval</i>");
    MCINFO("currentSpikeThresholdmA", "mA <i>Current spike = stall/endstop detected</i>");
    MCINFO("spikeSamplesRequired", "<i>Consecutive samples over threshold to trigger</i>");

    // LED control
    MCINFO("ledControlEnabled", "<i>Turn LEDs on/off based on lid position</i>");
    MCINFO("ledInvertDirection", "<i>Swap which direction is open vs closed</i>");
    MCINFO("ledSavedBri", "<i>Saved brightness restored when lid opens (persists across reboots)</i>");
    MCINFO("ledOffDistanceMm", "mm <i>LEDs turn off when within this distance of bottom (0 = off only at 0mm)</i>");

    #undef MCINFO
  }

  uint16_t getId() {
    return USERMOD_ID_MOTOR_CONTROLLER;
  }

  // ---------------------------
  // MQTT Command Handling (for Home Assistant integration)
  // ---------------------------
  void onMqttConnect(bool sessionPresent) override {
#ifndef WLED_DISABLE_MQTT
    // Subscribe to motor control topic: {mqttDeviceTopic}/motor/command
    if (mqttDeviceTopic[0] != 0) {
      char subuf[64];
      strcpy(subuf, mqttDeviceTopic);
      strcat_P(subuf, PSTR("/motor/command"));
      mqtt->subscribe(subuf, 0);

      // Publish initial state
      publishHomeAssistantSensor();
    }
#endif
  }

  bool onMqttMessage(char* topic, char* payload) override {
#ifndef WLED_DISABLE_MQTT
    // Topic comes pre-stripped of device prefix
    // We're looking for "/motor/command"
    if (strlen(topic) >= 14 && strncmp_P(topic, PSTR("/motor/command"), 14) == 0) {
      String action = payload;
      action.toLowerCase();

      if (action == "toggle" || action == "press") {
        // Toggle motor state (like pressing the button)
        if (motorState == IDLE) {
          beginStart();
        } else {
          beginStop(STOP_USER);
        }
        return true;
      } else if (action == "start" || action == "on") {
        // Start motor in current direction (only if idle)
        if (motorState == IDLE) {
          homeSeekActive = false;
          beginStart();
        }
        return true;
      } else if (action == "stop" || action == "off") {
        // Stop motor (only if running)
        if (motorState != IDLE) {
          beginStop(STOP_USER);
        }
        return true;
      } else if (action == "open") {
        // Move in opening direction (uses ledInvertDirection config)
        // ledInvertDirection: false = forward is opening, true = forward is closing
        if (motorState == IDLE) {
          homeSeekActive = false;
          motorDirection = !ledInvertDirection;  // Set to opening direction
          beginStart();
        }
        return true;
      } else if (action == "close") {
        // Move in closing direction (uses ledInvertDirection config)
        if (motorState == IDLE) {
          homeSeekActive = false;
          motorDirection = ledInvertDirection;  // Set to closing direction
          beginStart();
        }
        return true;
      } else if (action == "autohome") {
        // Auto-home: seek bottom by stalling, then back off and set home
        if (motorState == IDLE && !isHomed) {
          autoHomingActive = true;
          autoHomingPhaseBackoff = false;
          homeSeekActive = true;
          setDirectionDown();
          beginStart();
        }
        return true;
      } else if (action == "home" || action == "sethome") {
        // Set current position as home (0mm)
        if (motorState == IDLE) {
          isHomed = true;
          currentPositionMm = 0.0f;
          positionTicks = 0;
          setDirectionUp();
          publishHomeAssistantSensor();
        }
        return true;
      }
    }
#endif
    return false;
  }
};

// Instance and registration
static MotorControllerUsermod usermod_motor_controller;
REGISTER_USERMOD(usermod_motor_controller);
