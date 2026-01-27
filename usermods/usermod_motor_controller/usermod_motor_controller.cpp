#include "wled.h"
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <math.h>

/*
 * Motor Controller Usermod (WLED)
 * - PWM motor driver control (enablePin + IN1/IN2)
 * - Capacitive touch start/stop toggle
 * - Quadrature hall sensors for position tracking & direction detection
 * - INA219 current sensing to detect stall/endstop by current spike
 * - Configurable target travel distance with auto-stop
 *
 * Updates:
 * - Lower default PWM frequency (1kHz)
 * - S-curve acceleration/deceleration for gentle motor control
 * - Optional kickstart to overcome static friction
 * - Hall sensor direction detection (determines actual spin direction)
 * - Configurable distance per hall tick and target travel distance
 * - FIX for ESP32 "dangerous relocation: l32r" linker errors:
 *     Use a global free-function ISR + global volatile snapshot variables
 *     (avoid C++ member functions / instance access inside IRAM ISR)
 * - FIX: acceleration not working:
 *     Do NOT overwrite PWM with pwmMax in RUNNING state (we maintain currentPwm)
 * - FIX: spike detection only in one direction:
 *     Use fabsf(current_mA) so reversed current still triggers
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

void IRAM_ATTR hall_isr_capture() {
#if defined(ARDUINO_ARCH_ESP32)
  // GPIO32..39 are read from GPIO.in1.data (bit0 maps to GPIO32)
  const uint32_t in1 = (uint32_t)GPIO.in1.data;
  const uint8_t a = (in1 >> (g_hallAPin - 32)) & 0x1;
  const uint8_t b = (in1 >> (g_hallBPin - 32)) & 0x1;
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
  // Pin Configuration
  // ---------------------------
  int8_t enablePin = 25;  // PWM pin for speed control (ENA)
  int8_t in1Pin    = 26;  // Direction control 1 (IN1)
  int8_t in2Pin    = 27;  // Direction control 2 (IN2)
  int8_t touchPin  = 33;  // Capacitive touch sensor input (digital HIGH/LOW)

  // Hall sensors (quadrature)
  int8_t hallAPin  = 32;  // Hall A input
  int8_t hallBPin  = 35;  // Hall B input (GPIO35 is input-only, OK)

  // Optional: if you want to force I2C pins (ESP32 default usually 21/22)
  bool   i2cCustomPins = false;
  int8_t i2cSdaPin     = 21;
  int8_t i2cSclPin     = 22;

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
  int pwmChannel = 0;

  // Lower default frequency for many DC motor drivers
  int pwmFrequency = 1000; // was 5000
  int pwmResolution = 8;

  // Kickstart (helps overcome static friction / gearbox stiction)
  bool kickstartEnabled = true;
  int  kickstartPwm = 255;
  unsigned long kickstartMs = 120;

  // ---------------------------
  // Touch detection (noise immunity)
  // ---------------------------
  unsigned long capDebounceMs = 80;
  int touchSamples = 5;
  unsigned long touchLockoutMs = 300;

  // ---------------------------
  // Current sensing (INA219)
  // ---------------------------
  Adafruit_INA219 ina219;
  bool ina219Ok = false;

  // Polling interval for INA219 (ms)
  unsigned long currentPollMs = 50;
  unsigned long lastCurrentPoll = 0;

  // Spike detection
  float currentSpikeThresholdmA = 3500.0f; // adjust for your actuator + load
  uint8_t spikeSamplesRequired  = 3;       // consecutive samples over threshold
  uint8_t spikeSampleCount      = 0;

  // Exposed telemetry (signed, as INA219 may go negative when reversed)
  float lastCurrentmA = 0.0f;

  // ---------------------------
  // Position tracking (hall quadrature)
  // ---------------------------
  volatile int32_t positionTicks = 0;
  uint8_t lastAB = 0;

  // Distance configuration
  float distancePerTick = 1.0f;        // Distance units (e.g., mm) per hall sensor tick
  float targetDistance = 0.0f;         // Target distance to travel (0 = unlimited/manual stop)
  bool  targetDistanceEnabled = false; // Enable auto-stop at target distance

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

  enum StopReason { STOP_USER, STOP_SPIKE, STOP_TIMEOUT, STOP_TARGET_REACHED };
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
  bool ledControlEnabled = false;    // Enable LED on/off based on motor direction
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
    }
#endif
  }

  // ---------------------------
  // Motor control primitives
  // ---------------------------
  void setDirectionPins(bool forward) {
    if (forward) {
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, LOW);
    } else {
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, HIGH);
    }
  }

  void applyPwm(int pwmValue) {
    pwmValue = constrain(pwmValue, 0, 255);
    currentPwm = pwmValue;

#if defined(ARDUINO_ARCH_ESP32)
    ledcWrite(pwmChannel, pwmValue);
#else
    analogWrite(enablePin, pwmValue);
#endif
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
    // Coast
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    applyPwm(0);

    // Update LED state based on the direction we just finished
    // If we were opening (moving in opening direction), lid is now open -> LEDs ON
    // If we were closing (moving in closing direction), lid is now closed -> LEDs OFF
    bool wasOpening = isOpeningDirection(motorDirection);
    updateLedState(wasOpening);  // wasOpening means lid is now open

    motorState = IDLE;
    motorDirection = !motorDirection;

    spikeSampleCount = 0;
    publishHomeAssistantSensor();
  }

  void beginStart() {
    motorState = STARTING;
    runStartTime = millis();
    spikeSampleCount = 0;

    // Record starting position for distance tracking
    runStartTicks = positionTicks;

    // Reset detected direction at start
    detectedDirection = 0;
    directionConfidence = 0;

    setDirectionPins(motorDirection);

    // If we're starting to open the lid, turn LEDs on immediately
    if (isOpeningDirection(motorDirection)) {
      updateLedState(true);  // Lid is opening -> LEDs ON
    }

    // Start at pwmMin (note: some motors won't move until higher PWM)
    applyPwm(constrain(pwmMin, 0, 255));

    kickActive = false;
    if (kickstartEnabled && kickstartMs > 0) {
      kickActive = true;
      kickStartTime = millis();
      applyPwm(constrain(kickstartPwm, 0, 255));
    } else {
      rampStartTime = millis();
      rampStartPwm = currentPwm;
      rampTargetPwm = pwmMax;
    }

    publishHomeAssistantSensor();
  }

  void updateStartSequence() {
    const unsigned long now = millis();

    if (kickActive) {
      if (now - kickStartTime >= kickstartMs) {
        kickActive = false;

        // Drop to pwmMin, then ramp up
        applyPwm(constrain(pwmMin, 0, 255));
        rampStartTime = now;
        rampStartPwm = currentPwm;
        rampTargetPwm = pwmMax;
      } else {
        return; // keep kick PWM
      }
    }

    const unsigned long elapsed = now - rampStartTime;
    const int pwm = computeRampPwm(elapsed, accelTimeMs, rampStartPwm, rampTargetPwm);
    applyPwm(pwm);

    if (elapsed >= accelTimeMs) {
      // Do not force pwmMax elsewhere; currentPwm is now at target
      motorState = RUNNING;
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
    const unsigned long currentTime = millis();

    if (currentTime - lastTouchTime < touchLockoutMs) return false;

    const bool currentReading = digitalRead(touchPin);

    if (currentReading == HIGH) {
      if (touchSampleCount == 0) {
        touchStartTime = currentTime;
        touchSampleCount = 1;
      } else if (currentTime - touchStartTime < capDebounceMs) {
        touchSampleCount++;
        if (touchSampleCount >= touchSamples && !touchDetected) {
          touchDetected = true;
          lastTouchTime = currentTime;
          return true;
        }
      }
    } else {
      if (touchDetected) touchDetected = false;
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

      // Build confidence in detected direction
      if (delta == detectedDirection) {
        if (directionConfidence < 255) directionConfidence++;
      } else {
        // Direction changed - update if we see consistent change
        directionConfidence = 1;
        detectedDirection = delta;
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
  // Current sensing + stall detect
  // ---------------------------
  bool pollCurrentAndCheckSpike() {
    if (!ina219Ok) return false;

    const unsigned long now = millis();
    if (now - lastCurrentPoll < currentPollMs) return false;
    lastCurrentPoll = now;

    // Signed current (can go negative if current reverses relative to shunt)
    const float mA = ina219.getCurrent_mA();
    lastCurrentmA = mA;

    // Direction-agnostic spike detect
    const float abs_mA = fabsf(mA);

    if (abs_mA >= currentSpikeThresholdmA) {
      if (spikeSampleCount < 255) spikeSampleCount++;
    } else {
      spikeSampleCount = 0;
    }

    return (spikeSampleCount >= spikeSamplesRequired);
  }

public:
  void setup() {
    if (!enabled) return;

    // Motor pins
    pinMode(enablePin, OUTPUT);
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);

    // Touch input
    pinMode(touchPin, INPUT);

    // Hall inputs
    pinMode(hallAPin, INPUT_PULLUP);
    pinMode(hallBPin, INPUT_PULLUP);

    // Initialize lastAB (safe here)
    lastAB = ((uint8_t)digitalRead(hallAPin) << 1) | (uint8_t)digitalRead(hallBPin);

    // Configure ISR globals (must be set before attachInterrupt)
    g_hallAPin = (uint8_t)hallAPin;
    g_hallBPin = (uint8_t)hallBPin;

    // Attach interrupts using a FREE FUNCTION ISR (fixes linker errors)
    attachInterrupt(digitalPinToInterrupt(hallAPin), hall_isr_capture, CHANGE);
    attachInterrupt(digitalPinToInterrupt(hallBPin), hall_isr_capture, CHANGE);

#if defined(ARDUINO_ARCH_ESP32)
    // PWM LEDC setup
    ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
    ledcAttachPin(enablePin, pwmChannel);
#endif

    // Ensure motor is stopped on startup
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    applyPwm(0);

    // I2C + INA219
    if (i2cCustomPins) {
      Wire.begin(i2cSdaPin, i2cSclPin);
    } else {
      Wire.begin();
    }
    ina219Ok = ina219.begin();

    initDone = true;
  }

  void loop() {
    if (!enabled || !initDone || strip.isUpdating()) return;

    // Process hall updates frequently
    processHallIfPending();

    // Touch toggles start/stop
    if (capTouchPressed()) {
      if (motorState == IDLE) {
        beginStart();
      } else {
        beginStop(STOP_USER);
      }
    }

    // State machine
    if (motorState == STARTING) {
      if (millis() - runStartTime >= safetyMaxRunMs) {
        beginStop(STOP_TIMEOUT);
      } else {
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

      // Current spike => smooth auto-stop
      if (pollCurrentAndCheckSpike()) {
        beginStop(STOP_SPIKE);
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
                                                  "User";
      stopReason.add(reason);
    }

    if (!ina219Ok) {
      JsonArray warn = user.createNestedArray("INA219");
      warn.add("NOT DETECTED");
    }

    // LED control status
    if (ledControlEnabled) {
      JsonArray ledCtrl = user.createNestedArray("LED Control");
      ledCtrl.add(ledInvertDirection ? "Inverted" : "Normal");
    }

    // Add control button to simulate touch sensor
    JsonArray btn = user.createNestedArray(F("Motor Control"));
    String buttonHtml = F("<button class=\"btn btn-xs\" onclick=\"requestJson({motorController:{toggle:true}});\">");
    if (motorState == IDLE) {
      buttonHtml += F("<i class=\"icons off\">&#xe08f;</i> Start");
    } else {
      buttonHtml += F("<i class=\"icons on\">&#xe08f;</i> Stop");
    }
    buttonHtml += F("</button>");
    btn.add(buttonHtml);
    btn.add(motorDirection ? F(" (Next: Fwd)") : F(" (Next: Rev)"));
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
                                (lastStopReason == STOP_TARGET_REACHED) ? "target" : "user";

    // LED control state
    usermod["ledControlEnabled"] = ledControlEnabled;
    usermod["ledInvertDirection"] = ledInvertDirection;
  }

  void readFromJsonState(JsonObject& root) {
    JsonObject usermod = root["motorController"];
    if (usermod.isNull()) return;

    // Allow setting target distance via API
    if (!usermod["targetDistance"].isNull()) {
      targetDistance = usermod["targetDistance"].as<float>();
    }
    if (!usermod["targetEnabled"].isNull()) {
      targetDistanceEnabled = usermod["targetEnabled"].as<bool>();
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

    top["enablePin"] = enablePin;
    top["in1Pin"] = in1Pin;
    top["in2Pin"] = in2Pin;
    top["touchPin"] = touchPin;

    top["hallAPin"] = hallAPin;
    top["hallBPin"] = hallBPin;

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

    top["ina219_ok"] = ina219Ok;

    top["currentPollMs"] = currentPollMs;
    top["currentSpikeThresholdmA"] = currentSpikeThresholdmA;
    top["spikeSamplesRequired"] = spikeSamplesRequired;

    top["i2cCustomPins"] = i2cCustomPins;
    top["i2cSdaPin"] = i2cSdaPin;
    top["i2cSclPin"] = i2cSclPin;

    // LED control settings
    top["ledControlEnabled"] = ledControlEnabled;
    top["ledInvertDirection"] = ledInvertDirection;
  }

  bool readFromConfig(JsonObject& root) {
    // Defaults
    enabled = true;
    enablePin = 25;
    in1Pin = 26;
    in2Pin = 27;
    touchPin = 33;

    hallAPin = 32;
    hallBPin = 35;

    // Distance defaults
    distancePerTick = 1.0f;
    targetDistance = 0.0f;
    targetDistanceEnabled = false;
    ticksPerMm = 1.0f;

    accelTimeMs = 800;
    decelTimeMs = 800;
    safetyMaxRunMs = 20000;

    pwmMin = 0;
    pwmMax = 255;
    pwmFrequency = 1000;

    kickstartEnabled = true;
    kickstartPwm = 255;
    kickstartMs = 120;

    currentPollMs = 50;
    currentSpikeThresholdmA = 3500.0f;
    spikeSamplesRequired = 3;

    i2cCustomPins = false;
    i2cSdaPin = 21;
    i2cSclPin = 22;

    // LED control defaults
    ledControlEnabled = false;
    ledInvertDirection = false;

    JsonObject top = root["MotorController"];
    if (top.isNull()) return false;

    bool ok = true;
    ok &= getJsonValue(top["enabled"], enabled);

    ok &= getJsonValue(top["enablePin"], enablePin);
    ok &= getJsonValue(top["in1Pin"], in1Pin);
    ok &= getJsonValue(top["in2Pin"], in2Pin);
    ok &= getJsonValue(top["touchPin"], touchPin);

    ok &= getJsonValue(top["hallAPin"], hallAPin);
    ok &= getJsonValue(top["hallBPin"], hallBPin);

    // Distance configuration
    ok &= getJsonValue(top["distancePerTick"], distancePerTick);
    ok &= getJsonValue(top["targetDistance"], targetDistance);
    ok &= getJsonValue(top["targetDistanceEnabled"], targetDistanceEnabled);
    ok &= getJsonValue(top["ticksPerMm"], ticksPerMm); // legacy

    // Validate distancePerTick
    if (distancePerTick <= 0) distancePerTick = 1.0f;

    ok &= getJsonValue(top["accelTime"], accelTimeMs);
    ok &= getJsonValue(top["decelTime"], decelTimeMs);
    ok &= getJsonValue(top["safetyMaxRunMs"], safetyMaxRunMs);

    ok &= getJsonValue(top["pwmMin"], pwmMin);
    ok &= getJsonValue(top["pwmMax"], pwmMax);
    ok &= getJsonValue(top["pwmFrequency"], pwmFrequency);

    ok &= getJsonValue(top["kickstartEnabled"], kickstartEnabled);
    ok &= getJsonValue(top["kickstartPwm"], kickstartPwm);
    ok &= getJsonValue(top["kickstartMs"], kickstartMs);

    ok &= getJsonValue(top["currentPollMs"], currentPollMs);
    ok &= getJsonValue(top["currentSpikeThresholdmA"], currentSpikeThresholdmA);
    ok &= getJsonValue(top["spikeSamplesRequired"], spikeSamplesRequired);

    ok &= getJsonValue(top["i2cCustomPins"], i2cCustomPins);
    ok &= getJsonValue(top["i2cSdaPin"], i2cSdaPin);
    ok &= getJsonValue(top["i2cSclPin"], i2cSclPin);

    // LED control settings
    ok &= getJsonValue(top["ledControlEnabled"], ledControlEnabled);
    ok &= getJsonValue(top["ledInvertDirection"], ledInvertDirection);

    // Update ISR globals if pins changed via config (NOTE: interrupts already attached)
    g_hallAPin = (uint8_t)hallAPin;
    g_hallBPin = (uint8_t)hallBPin;

    // If PWM frequency changed via config, re-init LEDC
#if defined(ARDUINO_ARCH_ESP32)
    if (initDone) {
      ledcDetachPin(enablePin);
      ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
      ledcAttachPin(enablePin, pwmChannel);
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

    // Pin configuration section
    MCINFO("enablePin", "<i>PWM speed control (connect to ENA)</i>");
    MCINFO("in1Pin", "<i>Direction pin 1 (connect to IN1)</i>");
    MCINFO("in2Pin", "<i>Direction pin 2 (connect to IN2)</i>");
    MCINFO("touchPin", "<i>Capacitive touch or button input</i>");
    MCINFO("hallAPin", "<i>Quadrature encoder channel A</i>");
    MCINFO("hallBPin", "<i>Quadrature encoder channel B</i>");

    // Distance/position settings
    MCINFO("distancePerTick", "<i>Units traveled per encoder tick (e.g., mm)</i>");
    MCINFO("targetDistance", "<i>Auto-stop distance (0 = disabled)</i>");
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

    // Current sensing
    MCINFO("currentPollMs", "ms <i>INA219 polling interval</i>");
    MCINFO("currentSpikeThresholdmA", "mA <i>Current spike = endstop reached</i>");
    MCINFO("spikeSamplesRequired", "<i>Consecutive samples over threshold to trigger</i>");

    // I2C settings
    MCINFO("i2cCustomPins", "<i>Use custom I2C pins instead of defaults</i>");
    MCINFO("i2cSdaPin", "<i>Custom I2C data pin</i>");
    MCINFO("i2cSclPin", "<i>Custom I2C clock pin</i>");

    // LED control
    MCINFO("ledControlEnabled", "<i>Turn LEDs on/off based on lid position</i>");
    MCINFO("ledInvertDirection", "<i>Swap which direction is open vs closed</i>");

    #undef MCINFO
  }

  uint16_t getId() {
    return USERMOD_ID_MOTOR_CONTROLLER;
  }
};

// Instance and registration
static MotorControllerUsermod usermod_motor_controller;
REGISTER_USERMOD(usermod_motor_controller);
