# WLED Linear Actuator Motor Controller Usermod

This WLED usermod adds closed-loop control for a high-current DC linear actuator (such as the Vevor TV lift using the ZYT45JS-2 motor).

It provides:
- Smooth S-curve PWM acceleration/deceleration for gentle motor control
- Capacitive touch start/stop with direction toggle
- Dual hall-effect sensor position tracking (quadrature)
- Hall sensor direction detection (determines actual motor spin direction)
- INA219 current sensing for stall / end-of-travel detection
- Configurable distance per hall tick
- Configurable target travel distance with auto-stop
- Automatic stop and direction reversal when endstop is reached
- Position, distance, and current reporting via WLED JSON API and MQTT

This allows a single capacitive button to safely drive a linear actuator up and down while automatically stopping when mechanical limits are reached.

---

## Features

### Motion Control
- S-curve (smootherstep) acceleration for gentle starts
- S-curve deceleration for smooth stops
- Configurable acceleration/deceleration time
- Full-speed cruise
- Safety timeout
- Immediate stop on second button press
- Direction toggles after each stop
- Optional kickstart pulse to overcome static friction

### Position & Distance Tracking
- Two hall sensors decoded as quadrature
- 32-bit tick counter
- Configurable distance per tick (mm, inches, or any unit)
- Real-time distance traveled calculation
- Target distance auto-stop (motor stops when configured distance is reached)
- Direction-aware counting

### Hall Sensor Direction Detection
- Detects actual motor spin direction from hall sensor signals
- Independent of commanded direction (detects mechanical reality)
- Confidence indicator for direction detection
- Useful for detecting slippage or incorrect wiring

### Current-Based Endstop Detection
- INA219 high-side current sensor
- Stall detection by sustained current spike
- Adjustable threshold and sample count
- Prevents gear damage and motor overheating

### Integration
WLED JSON API state fields (read):
- `running` - true if motor is active
- `state` - "idle", "starting", "running", "stopping"
- `direction` - commanded direction: "forward" or "reverse"
- `hallDirection` - detected direction from hall sensors: "forward", "reverse", or "unknown"
- `hallConfidence` - confidence level of detected direction (0-255)
- `posTicks` - position in hall sensor ticks
- `distance` - distance traveled in current run (in configured units)
- `targetDistance` - configured target distance
- `targetEnabled` - whether target distance auto-stop is enabled
- `current_mA` - motor current from INA219
- `pwm` - current PWM duty value
- `lastStopReason` - "user", "spike", "timeout", or "target"

WLED JSON API state fields (write):
- `start` - set to true to start motor
- `stop` - set to true to stop motor
- `resetPos` - set to true to reset position to zero
- `targetDistance` - set target distance
- `targetEnabled` - enable/disable target distance auto-stop
- `distancePerTick` - set distance per hall tick

Optional MQTT telemetry for Home Assistant

---

## Hardware Requirements

### Motor & Driver
- 18V linear actuator (e.g. Vevor TV Lift, ZYT45JS-2 motor)
- H-bridge motor driver (BTS7960 / IBT-2 / equivalent, ≥10A)

### Sensors
- 2x Hall effect outputs from actuator (quadrature A/B)
- INA219 current sensor module (I²C)

### Controller
- ESP32 running WLED (GPIO interrupts required)

### Example Pin Assignment

| Function | ESP32 Pin |
|---------|-----------|
| PWM (ENA) | GPIO 25 |
| IN1 | GPIO 26 |
| IN2 | GPIO 27 |
| Touch Button | GPIO 33 |
| Hall A | GPIO 32 |
| Hall B | GPIO 35 |
| I²C SDA | GPIO 21 |
| I²C SCL | GPIO 22 |

---

## Behavior

1. Tap capacitive button:
   - Motor starts moving in current direction
   - Acceleration ramp applied
2. While moving:
   - Hall sensors track position
   - INA219 monitors motor current
3. If current spikes (stall / endstop):
   - Motor stops automatically
   - Direction toggles for next press
4. Tap while moving:
   - Immediate stop
   - Direction toggles

---

## Calibration (Distance per Tick)

Distance is derived from hall sensor ticks. To calibrate:

1. Reset position to zero via API: `{"motorController": {"resetPos": true}}`
2. Move actuator a known distance (e.g. 100mm).
3. Read `posTicks` from JSON state.
4. Compute distance per tick:

```
distancePerTick = distance_mm / ticks
```

For example, if 100mm = 12800 ticks:
```
distancePerTick = 100 / 12800 = 0.0078125
```

Set this value in WLED config:
```json
"distancePerTick": 0.0078125
```

## Target Distance Configuration

To make the motor automatically stop after traveling a specific distance:

1. Set `distancePerTick` as described above
2. Configure the target:

Via WLED Config UI:
- Set `targetDistance` (e.g. 500 for 500mm)
- Set `targetDistanceEnabled` to true

Via JSON API:
```json
{"motorController": {"targetDistance": 500, "targetEnabled": true, "start": true}}
```

The motor will automatically stop with smooth deceleration when the target distance is reached.

---

## S-Curve Acceleration

The motor uses a smootherstep S-curve for acceleration and deceleration, providing gentle starts and stops that reduce mechanical stress and jerky motion.

The S-curve formula (Ken Perlin's smootherstep):
```
f(t) = 6t^5 - 15t^4 + 10t^3
```

This provides:
- Zero velocity at start and end
- Zero acceleration at start and end
- Smooth, natural-feeling motion

Configuration:
- `accelTime` - time in ms for acceleration ramp (default: 800ms)
- `decelTime` - time in ms for deceleration ramp (default: 800ms)
- `pwmMin` - starting PWM value (default: 0)
- `pwmMax` - target PWM value (default: 255)

Optional kickstart (helps overcome static friction):
- `kickstartEnabled` - enable kickstart pulse (default: true)
- `kickstartPwm` - PWM value for kickstart (default: 255)
- `kickstartMs` - duration of kickstart in ms (default: 120ms)

---

## Current Spike Tuning

Defaults:
- currentSpikeThresholdmA = 3500.0
- spikeSamplesRequired = 3

Tune by:
- Logging current during free motion
- Logging during stall
- Set threshold ~70–80% of stall current
- Require 2–4 consecutive samples

---

## Building (PlatformIO / GitHub Actions)

This usermod requires the following library:

Adafruit INA219

Add to your PlatformIO environment:

lib_deps =
  adafruit/Adafruit INA219

---

## Safety Notes

- Always use a motor driver rated well above stall current.
- INA219 must be wired high-side for accurate PWM current sensing.
- Do not rely on software only for mechanical safety — physical endstops are still recommended.
- This system is designed to detect stalls, not prevent all possible damage scenarios.

---

## License

MIT