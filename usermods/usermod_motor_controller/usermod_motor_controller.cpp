#include "wled.h"
#include <math.h>

/*
 * Motor Controller Usermod (WLED)
 * - BTS7960 dual half-bridge motor driver (RPWM/LPWM + R_EN/L_EN)
 * - Capacitive touch start/stop toggle
 * - Quadrature hall sensors for position tracking & direction detection
 * - BTS7960 IS pin current sensing to detect stall/endstop by current spike
 * - Configurable target travel distance with auto-stop
 * - Bottom endstop switch for homing & absolute position tracking
 * - Firmware travel limit (18" / 457.2mm) to protect linear actuator stroke
 * - Homing sequence: motor must find bottom endstop before allowing upward travel
 * - Safe boot: motor never moves automatically on power-up
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
static volatile bool    g_hallPending    = false;
static volatile uint8_t g_hallABSnapshot = 0;
static volatile int8_t  g_hallLastDelta  = 0;  // Last direction detected: +1 forward, -1 reverse, 0 no change
static uint8_t          g_hallAPin       = 32;
static uint8_t          g_hallBPin       = 35;

#if defined(ARDUINO_ARCH_ESP32)
static inline uint8_t IRAM_ATTR read_gpio_pin_iram(uint8_t pin) {
  if (pin < 32) return (uint8_t)((GPIO.in >> pin) & 0x1);
  if (pin < 40) return (uint8_t)((GPIO.in1.data >> (pin - 32)) & 0x1);
  return 0;
}
#endif

void IRAM_ATTR hall_isr_capture() {
#if defined(ARDUINO_ARCH_ESP32)
  // Read from low/high GPIO input register based on pin number.
  const uint8_t a = read_gpio_pin_iram(g_hallAPin);
  const uint8_t b = read_gpio_pin_iram(g_hallBPin);
  g_hallABSnapshot = (a << 1) | b;
  g_hallPending = true;
#else
  // Fallback (not IRAM-safe on non-ESP32), but keeps code portable.
  g_hallABSnapshot = ((uint8_t)digitalRead(g_hallAPin) << 1) | (uint8_t)digitalRead(g_hallBPin);
  g_hallPending = true;
#endif
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
  int8_t renPin  = 12;   // BTS7960 R_EN — enable extend half-bridge
  int8_t lenPin  = 13;   // BTS7960 L_EN — enable retract half-bridge
  int8_t touchPin = 33;  // Capacitive touch sensor input (digital HIGH/LOW)

  // Bottom endstop switch
  int8_t endstopPin = 22;          // Endstop input
  bool   endstopActiveLow = true;  // true = LOW when triggered (pull-up + NO switch to GND)
  bool   endstopEnabled = true;    // Enable endstop functionality
  unsigned long endstopDebounceMs = 20;  // Debounce time for mechanical switch (ms)
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
  unsigned long accelTimeMs = 800;
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
  // Touch detection (noise immunity)
  // ---------------------------
  unsigned long capDebounceMs = 80;
  int touchSamples = 5;
  unsigned long touchLockoutMs = 300;

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
  uint8_t lastAB = 0;

  // Distance configuration
  float distancePerTick = 1.0f;        // Distance units (e.g., mm) per hall sensor tick
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

  // Legacy alias for backwards compatibility
  float ticksPerMm = 1.0f; // placeholder (use distancePerTick instead)

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

  // ---------------------------
  // Travel Limits
  // ---------------------------
  // 18 inches = 457.2mm — firmware hard limit (actuator has 35" stroke, this keeps well within)
  static constexpr float FIRMWARE_MAX_TRAVEL_MM = 457.2f;
  float maxTravelDistance = 457.2f;  // User-adjustable max travel, clamped to firmware limit

  // ---------------------------
  // Motor Motion State Machine
  // ---------------------------
  enum MotorState { IDLE, STARTING, RUNNING, STOPPING };
  MotorState motorState = IDLE;

  bool motorDirection = true; // true = forward/up, false = reverse/down
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

  // Touch detection state
  int touchSampleCount = 0;
  bool touchDetected = false;
  unsigned long lastTouchTime = 0;
  unsigned long touchStartTime = 0;

  // Enable/disable usermod
  bool enabled = true;
  bool initDone = false;

  // ---------------------------
  // LED Control (based on lid state)
  // ---------------------------
  bool ledControlEnabled = true;     // Enable LED on/off based on motor direction
  bool ledInvertDirection = false;   // false: forward=opening (LEDs on), true: forward=closing (LEDs off)
  uint8_t ledSavedBri = 128;         // Saved brightness to restore when turning LEDs back on

  // ---------------------------
  // LED Control (turn on/off based on lid state)
  // ---------------------------
  void updateLedState(bool lidIsOpen) {
    if (!ledControlEnabled) return;

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

  // Determine if moving in "opening" direction based on config
  bool isOpeningDirection(bool direction) {
    // direction: true = forward, false = reverse
    // ledInvertDirection: false = forward is opening, true = forward is closing
    return ledInvertDirection ? !direction : direction;
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

  // Update debounced endstop reading — must be called every loop iteration
  void updateEndstopDebounce() {
    if (!endstopEnabled) return;
    bool rawState = digitalRead(endstopPin) == (endstopActiveLow ? LOW : HIGH);
    if (rawState != lastRawEndstopState) {
      lastRawEndstopState = rawState;
      lastEndstopChangeTime = millis();
    }
    if (millis() - lastEndstopChangeTime >= endstopDebounceMs) {
      debouncedEndstopState = lastRawEndstopState;
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
    coastMotor();

    lastStopReason = reason;
    motorState = IDLE;
    kickActive = false;
    spikeSampleCount = 0;

    bool wasOpening = isOpeningDirection(motorDirection);
    updateLedState(wasOpening);

    if (toggleDirectionAfter) {
      motorDirection = !motorDirection;
    }

    publishHomeAssistantSensor();
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

    // Update LED state based on the direction we just finished
    // If we were opening (moving in opening direction), lid is now open -> LEDs ON
    // If we were closing (moving in closing direction), lid is now closed -> LEDs OFF
    bool wasOpening = isOpeningDirection(motorDirection);
    updateLedState(wasOpening);  // wasOpening means lid is now open

    motorState = IDLE;

    // Always toggle direction — beginStart() handles safety overrides
    // (e.g., forcing DOWN when unhomed, redirecting at endstop)
    motorDirection = !motorDirection;

    spikeSampleCount = 0;
    publishHomeAssistantSensor();
  }

  void beginStart() {
    // ---------------------------
    // Pre-start safety checks
    // ---------------------------
    if (endstopEnabled) {
      // If not homed, force direction DOWN to find home first
      if (!isHomed) {
        setDirectionDown();
      }

      // If trying to go DOWN but endstop is already triggered,
      // redirect to UP and continue (don't silently consume the touch press)
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

    // Record starting position for distance tracking
    runStartTicks = positionTicks;

    // Reset detected direction at start
    detectedDirection = 0;
    directionConfidence = 0;

    // Enable BTS7960 half-bridges and set direction (PWM routing)
    enableMotorDriver();
    setDirectionPins(motorDirection);

    // If we're starting to open the lid, turn LEDs on immediately
    if (isOpeningDirection(motorDirection)) {
      updateLedState(true);  // Lid is opening -> LEDs ON
    }

    // Start at pwmMin (note: some motors won't move until higher PWM)
    applyPwm(constrain(pwmMin, 0, 255));

    kickActive = false;
    rampStartTime = millis();
    rampStartPwm = currentPwm;
    rampTargetPwm = pwmMax;

    publishHomeAssistantSensor();
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

    const int pwm = computeRampPwm(elapsed, decelTimeMs, rampStartPwm, 0);
    applyPwm(pwm);

    if (elapsed >= decelTimeMs) {
      finalizeStopAndToggleDirection();
    }
  }

  // ---------------------------
  // Touch detection
  // ---------------------------
  bool capTouchPressed() {
    const unsigned long now = millis();

    // Lockout prevents rapid re-triggers after a confirmed press
    if (now - lastTouchTime < touchLockoutMs) return false;

    const bool reading = digitalRead(touchPin) == HIGH;

    if (reading) {
      if (touchSampleCount == 0) {
        // First HIGH reading — start debounce timer
        touchStartTime = now;
        touchSampleCount = 1;
      }
      // Signal has been continuously HIGH since touchStartTime
      if (!touchDetected && (now - touchStartTime >= capDebounceMs)) {
        // Stable HIGH for capDebounceMs — confirmed press
        touchDetected = true;
        lastTouchTime = now;
        return true;
      }
    } else {
      // Signal LOW — reset detection state
      touchDetected = false;
      touchSampleCount = 0;
    }

    return false;
  }

  // ---------------------------
  // Hall processing (decode in loop)
  // ---------------------------
  void processHallIfPending() {
    if (!g_hallPending) return;

    uint8_t ab;
    noInterrupts();
    ab = g_hallABSnapshot;
    g_hallPending = false;
    interrupts();

    // Quadrature decode table
    // Maps (lastAB << 2 | currentAB) to direction: +1 forward, -1 reverse, 0 no movement
    static const int8_t qdec[16] = {
      0, -1,  1,  0,
      1,  0,  0, -1,
     -1,  0,  0,  1,
      0,  1, -1,  0
    };

    const int8_t delta = qdec[(lastAB << 2) | ab];
    lastAB = ab;

    positionTicks += delta;

    // Update detected direction from hall sensors
    if (delta != 0) {
      g_hallLastDelta = delta;
      lastHallTickTime = millis();  // Track for stall detection

      // Build confidence in detected direction
      if (delta == detectedDirection) {
        if (directionConfidence < 255) directionConfidence++;
      } else {
        // Direction changed - update if we see consistent change
        directionConfidence = 1;
        detectedDirection = delta;
      }

      // Update absolute position when homed and motor is active
      if (isHomed && motorState != IDLE) {
        float tickDistanceMm = fabsf((float)delta * distancePerTick);
        if (isMovingUp()) {
          currentPositionMm += tickDistanceMm;
        } else {
          currentPositionMm -= tickDistanceMm;
          if (currentPositionMm < 0.0f) currentPositionMm = 0.0f;
        }
      }
    }
  }

  // Get the distance traveled since the start of the current run
  float getDistanceTraveled() {
    int32_t ticksDelta = positionTicks - runStartTicks;
    // Use absolute value since we care about distance magnitude
    if (ticksDelta < 0) ticksDelta = -ticksDelta;
    return (float)ticksDelta * distancePerTick;
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

    // GPIO12 is a strapping pin — must be LOW during/after boot or ESP32 may fail to start.
    // Explicitly drive enables LOW immediately before any other setup.
    digitalWrite(renPin, LOW);
    digitalWrite(lenPin, LOW);
    delay(100);  // Allow strapping pin to stabilize after boot

    // Touch input
    pinMode(touchPin, INPUT);

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
    }

    // Hall inputs (no internal pullup — HE sensor board provides external pullups)
    pinMode(hallAPin, INPUT);
    pinMode(hallBPin, INPUT);

    // Initialize lastAB (safe here)
    lastAB = ((uint8_t)digitalRead(hallAPin) << 1) | (uint8_t)digitalRead(hallBPin);

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
        // Lid is closed/down — LEDs should be OFF
        updateLedState(false);
      } else {
        // Endstop NOT triggered — position is unknown, needs homing.
        // First user-initiated movement will go DOWN to find home.
        // Motor will NOT move automatically (safety: prevent pinch on power-up).
        isHomed = false;
        currentPositionMm = 0.0f;
        setDirectionDown();  // Force first movement toward endstop
        // Position unknown — default LEDs OFF for safety
        updateLedState(false);
      }
    }

    initDone = true;
  }

  void loop() {
    if (!enabled || !initDone) return;

    // Motor control must run every iteration regardless of strip updates.
    // GPIO reads, PWM writes, and touch detection are safe during DMA.

    // Process hall updates frequently
    processHallIfPending();

    // Update debounced endstop state every iteration
    updateEndstopDebounce();

    // ---------------------------
    // Endstop monitoring (every loop iteration for safety)
    // ---------------------------
    if (endstopEnabled) {
      bool endstopHit = isEndstopTriggered();

      // If endstop triggers while motor is moving DOWN — IMMEDIATE stop
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
    // Travel limit check while motor is running UP
    // ---------------------------
    // Only trigger travel-limit stop while actively accelerating/running.
    // If we are already STOPPING, allow the stop sequence to finish;
    // otherwise we can starve touch handling and never finalize the stop.
    if (endstopEnabled && isHomed &&
        (motorState == STARTING || motorState == RUNNING) &&
        isMovingUp()) {
      if (currentPositionMm >= maxTravelDistance) {
        beginStop(STOP_TRAVEL_LIMIT);
        return;
      }
    }

    // Touch toggles start/stop
    if (capTouchPressed()) {
      if (motorState == IDLE) {
        beginStart();
      } else if (motorState == STOPPING) {
        // Already decelerating — cut power immediately on touch
        finalizeStopAndToggleDirection();
      } else {
        beginStop(STOP_USER);
      }
    }

    // State machine
    if (motorState == STARTING) {
      if (millis() - runStartTime >= safetyMaxRunMs) {
        beginStop(STOP_TIMEOUT);
      } else {
        const unsigned long runElapsed = millis() - runStartTime;
        // Fast-fault path: hard stop on encoder stall/current spike even during ramp-up.
        if (stallDetectionEnabled && runElapsed >= stallStartGraceMs &&
            (millis() - lastHallTickTime > stallTimeoutMs)) {
          immediateStop(STOP_STALL, true);
          return;
        }
        if (pollCurrentAndCheckSpike()) {
          immediateStop(STOP_SPIKE, true);
          return;
        }
        updateStartSequence();
      }
      return;
    }

    if (motorState == RUNNING) {
      const unsigned long elapsed = millis() - runStartTime;

      // Safety timeout
      if (elapsed >= safetyMaxRunMs) {
        beginStop(STOP_TIMEOUT);
        return;
      }

      // Target distance reached => smooth auto-stop
      if (hasReachedTargetDistance()) {
        beginStop(STOP_TARGET_REACHED);
        return;
      }

      // Encoder stall detection (Safety Layer 2): no hall tick in stallTimeoutMs
      if (stallDetectionEnabled && (millis() - lastHallTickTime > stallTimeoutMs)) {
        immediateStop(STOP_STALL, true);
        return;
      }

      // Current spike => immediate hard stop for minimum fault latency.
      if (pollCurrentAndCheckSpike()) {
        immediateStop(STOP_SPIKE, true);
        return;
      }

      // Maintain direction and PWM (DO NOT overwrite with pwmMax here,
      // otherwise it cancels acceleration behavior)
      setDirectionPins(motorDirection);
      applyPwm(currentPwm); // typically pwmMax after ramp completes
      return;
    }

    if (motorState == STOPPING) {
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

    // Homing / endstop status
    if (endstopEnabled) {
      JsonArray homeStatus = user.createNestedArray("Homed");
      homeStatus.add(isHomed ? "Yes" : "NO - press button to home");

      JsonArray endstopStatus = user.createNestedArray("Endstop");
      endstopStatus.add(isEndstopTriggered() ? "TRIGGERED" : "Open");

      if (isHomed) {
        JsonArray absPos = user.createNestedArray("Position (mm)");
        absPos.add(currentPositionMm);
        absPos.add(" / ");
        absPos.add(maxTravelDistance);
      }
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

    // LED control status
    if (ledControlEnabled) {
      JsonArray ledCtrl = user.createNestedArray("LED Control");
      ledCtrl.add(ledInvertDirection ? "Inverted" : "Normal");
    }

    // Add control buttons
    JsonArray btn = user.createNestedArray(F("Motor Control"));
    String buttonHtml = F("<button class=\"btn btn-xs\" onclick=\"requestJson({motorController:{toggle:true}});\">");
    if (motorState == IDLE) {
      if (endstopEnabled && !isHomed) {
        buttonHtml += F("<i class=\"icons off\">&#xe08f;</i> Home");
      } else {
        buttonHtml += F("<i class=\"icons off\">&#xe08f;</i> Start");
      }
    } else {
      buttonHtml += F("<i class=\"icons on\">&#xe08f;</i> Stop");
    }
    buttonHtml += F("</button>");

    // Add dedicated Home button when endstop is enabled
    if (endstopEnabled) {
      buttonHtml += F(" <button class=\"btn btn-xs\" onclick=\"requestJson({motorController:{home:true}});\">");
      buttonHtml += F("<i class=\"icons\">&#xe08f;</i> Home");
      buttonHtml += F("</button>");
    }

    // Fine control: press-and-hold jog buttons.
    // Press starts movement; release/cancel/leave sends stop.
    buttonHtml += F(" <button class=\"btn btn-xs\" onpointerdown=\"requestJson({motorController:{open:true}});\" ");
    buttonHtml += F("onpointerup=\"requestJson({motorController:{stop:true}});\" ");
    buttonHtml += F("onpointercancel=\"requestJson({motorController:{stop:true}});\" ");
    buttonHtml += F("onmouseleave=\"requestJson({motorController:{stop:true}});\" ");
    buttonHtml += F("ontouchend=\"requestJson({motorController:{stop:true}});\">");
    buttonHtml += F("<i class=\"icons\">&#xe00f;</i> Up");
    buttonHtml += F("</button>");

    buttonHtml += F(" <button class=\"btn btn-xs\" onpointerdown=\"requestJson({motorController:{close:true}});\" ");
    buttonHtml += F("onpointerup=\"requestJson({motorController:{stop:true}});\" ");
    buttonHtml += F("onpointercancel=\"requestJson({motorController:{stop:true}});\" ");
    buttonHtml += F("onmouseleave=\"requestJson({motorController:{stop:true}});\" ");
    buttonHtml += F("ontouchend=\"requestJson({motorController:{stop:true}});\">");
    buttonHtml += F("<i class=\"icons\">&#xe010;</i> Down");
    buttonHtml += F("</button>");

    btn.add(buttonHtml);
    if (endstopEnabled && !isHomed) {
      btn.add(F(" (Needs homing)"));
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

    // Allow setting distance per tick via API
    if (!usermod["distancePerTick"].isNull()) {
      distancePerTick = usermod["distancePerTick"].as<float>();
      if (distancePerTick <= 0) distancePerTick = 1.0f;
    }

    // Toggle (simulates touch sensor press)
    if (usermod["toggle"].as<bool>()) {
      if (motorState == IDLE) {
        beginStart();
      } else {
        beginStop(STOP_USER);
      }
    }

    // Explicit start/stop commands
    if (usermod["start"].as<bool>() && motorState == IDLE) {
      beginStart();
    }

    if (usermod["stop"].as<bool>() && motorState != IDLE) {
      beginStop(STOP_USER);
    }

    // Direction-aware open/close commands
    // Uses ledInvertDirection to determine which physical direction is "open"
    if (usermod["open"].as<bool>() && motorState == IDLE) {
      motorDirection = !ledInvertDirection;  // Set to opening direction
      beginStart();
    }

    if (usermod["close"].as<bool>() && motorState == IDLE) {
      motorDirection = ledInvertDirection;  // Set to closing direction
      beginStart();
    }

    // Home command: move DOWN to find endstop
    if (usermod["home"].as<bool>() && motorState == IDLE && endstopEnabled) {
      if (isEndstopTriggered()) {
        // Already at home
        isHomed = true;
        currentPositionMm = 0.0f;
        positionTicks = 0;
        setDirectionUp();
      } else {
        isHomed = false;
        setDirectionDown();
        beginStart();
      }
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
    top["touchPin"] = touchPin;

    top["hallAPin"] = hallAPin;
    top["hallBPin"] = hallBPin;

    // Endstop configuration
    top["endstopPin"] = endstopPin;
    top["endstopActiveLow"] = endstopActiveLow;
    top["endstopEnabled"] = endstopEnabled;
    top["endstopDebounceMs"] = endstopDebounceMs;

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
    top["distancePerTick"] = distancePerTick;
    top["targetDistance"] = targetDistance;
    top["targetDistanceEnabled"] = targetDistanceEnabled;
    top["ticksPerMm"] = ticksPerMm; // legacy alias

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
  }

  bool readFromConfig(JsonObject& root) {
    // Defaults
    enabled = true;
    rpwmPin = 27;
    lpwmPin = 14;
    renPin = 12;
    lenPin = 13;
    touchPin = 33;

    hallAPin = 32;
    hallBPin = 35;

    // Endstop defaults
    endstopPin = 22;
    endstopActiveLow = true;
    endstopEnabled = true;
    endstopDebounceMs = 20;

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
    distancePerTick = 1.0f;
    targetDistance = FIRMWARE_MAX_TRAVEL_MM;
    targetDistanceEnabled = true;
    ticksPerMm = 1.0f;

    accelTimeMs = 800;
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

    JsonObject top = root["MotorController"];
    if (top.isNull()) return false;

    bool ok = true;
    ok &= getJsonValue(top["enabled"], enabled);

    ok &= getJsonValue(top["rpwmPin"], rpwmPin);
    ok &= getJsonValue(top["lpwmPin"], lpwmPin);
    ok &= getJsonValue(top["renPin"], renPin);
    ok &= getJsonValue(top["lenPin"], lenPin);
    ok &= getJsonValue(top["touchPin"], touchPin);

    ok &= getJsonValue(top["hallAPin"], hallAPin);
    ok &= getJsonValue(top["hallBPin"], hallBPin);
    if (hallAPin < 0 || hallAPin > 39) hallAPin = 32;
    if (hallBPin < 0 || hallBPin > 39) hallBPin = 35;

    // Endstop configuration
    ok &= getJsonValue(top["endstopPin"], endstopPin);
    ok &= getJsonValue(top["endstopActiveLow"], endstopActiveLow);
    ok &= getJsonValue(top["endstopEnabled"], endstopEnabled);
    ok &= getJsonValue(top["endstopDebounceMs"], endstopDebounceMs);

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
    ok &= getJsonValue(top["distancePerTick"], distancePerTick);
    ok &= getJsonValue(top["targetDistance"], targetDistance);
    ok &= getJsonValue(top["targetDistanceEnabled"], targetDistanceEnabled);
    ok &= getJsonValue(top["ticksPerMm"], ticksPerMm); // legacy

    // Validate distancePerTick
    if (distancePerTick <= 0) distancePerTick = 1.0f;

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
    MCINFO("touchPin", "<i>Capacitive touch or button input</i>");
    MCINFO("hallAPin", "<i>Quadrature encoder channel A</i>");
    MCINFO("hallBPin", "<i>Quadrature encoder channel B</i>");

    // Endstop settings
    MCINFO("endstopPin", "<i>Bottom endstop switch input (GPIO34-39 need external pull-up)</i>");
    MCINFO("endstopActiveLow", "<i>true = LOW when triggered (NO switch + pull-up)</i>");
    MCINFO("endstopEnabled", "<i>Enable bottom endstop for homing &amp; safety</i>");
    MCINFO("endstopDebounceMs", "ms <i>Endstop switch debounce time (10-50ms for mechanical)</i>");

    // Direction & travel
    MCINFO("downIsForward", "<i>false = reverse is DOWN (toward endstop)</i>");
    MCINFO("invertMotorDirection", "<i>Swap RPWM/LPWM logic if motor runs backwards</i>");

    // Stall detection
    MCINFO("stallDetectionEnabled", "<i>Stop motor if encoder reports no movement</i>");
    MCINFO("stallTimeoutMs", "ms <i>No encoder pulse for this long = stalled (100-300)</i>");
    MCINFO("stallStartGraceMs", "ms <i>Grace period after start before stall checks begin</i>");
    MCINFO("maxTravelDistance", "mm <i>Max UP travel from home (firmware limit: 457.2mm / 18&quot;)</i>");

    // Distance/position settings
    MCINFO("distancePerTick", "<i>Units traveled per encoder tick (e.g., mm)</i>");
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
        // Toggle motor state (like pressing the touch button)
        if (motorState == IDLE) {
          beginStart();
        } else {
          beginStop(STOP_USER);
        }
        return true;
      } else if (action == "start" || action == "on") {
        // Start motor in current direction (only if idle)
        if (motorState == IDLE) {
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
          motorDirection = !ledInvertDirection;  // Set to opening direction
          beginStart();
        }
        return true;
      } else if (action == "close") {
        // Move in closing direction (uses ledInvertDirection config)
        if (motorState == IDLE) {
          motorDirection = ledInvertDirection;  // Set to closing direction
          beginStart();
        }
        return true;
      } else if (action == "home") {
        // Home: move down to find endstop
        if (motorState == IDLE && endstopEnabled) {
          if (isEndstopTriggered()) {
            isHomed = true;
            currentPositionMm = 0.0f;
            positionTicks = 0;
            setDirectionUp();
            publishHomeAssistantSensor();
          } else {
            isHomed = false;
            setDirectionDown();
            beginStart();
          }
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
