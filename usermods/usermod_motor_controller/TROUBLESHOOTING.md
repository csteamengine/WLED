# Motor Not Moving: Quick Troubleshooting

Use this when the motor is powered but does not move when you press touch/UI buttons.

## 1. Quick Sanity Checks (1-2 minutes)

- Confirm usermod is enabled in WLED config:
  - `MotorController.enabled = true`
- Confirm motor state changes when you press a control:
  - In JSON/state, check `motorController.state` goes `idle -> starting/running`.
- Confirm you are homed (or intentionally homing first):
  - If `endstopEnabled = true` and `isHomed = false`, first movement is forced DOWN to find endstop.
- Try explicit commands instead of toggle:
  - `open` and `close` (or hold `Up` / `Down` buttons) to verify direction behavior.

If state never leaves `idle`, focus on input/UI command path first.
If state enters `starting/running` but motor is still still, continue below.

## 2. Settings To Tweak First (in this order)

1. **Direction mapping**
- `downIsForward` (flip this first if movement logic seems inverted)
- `invertMotorDirection` (flip if motor wiring/direction is opposite expected)

2. **PWM / startup torque**
- Raise `pwmMin` (example: `0 -> 80 -> 120`)
- Keep `pwmMax = 255` during troubleshooting
- Keep `kickstartEnabled = false` first, then try:
  - `kickstartEnabled = true`
  - `kickstartPwm = 255`
  - `kickstartMs = 100-200`

3. **Fault trips too aggressive**
- Temporarily relax fault sensitivity to test motion:
  - Increase `currentSpikeThresholdmA`
  - Increase `stallTimeoutMs` (example: `150 -> 300`)
  - Increase `stallStartGraceMs` (example: `100 -> 250`)
- Watch `lastStopReason` after each attempt (`spike`, `stall`, `endstop`, etc.)

4. **Travel/endstop logic**
- If motor only refuses one direction:
  - Check `endstopTriggered`
  - Check `currentPositionMm` vs `maxTravelDistance`
- Temporarily set:
  - `targetDistanceEnabled = false` (for diagnosis only)
  - Keep `endstopEnabled = true` for safety

## 3. What To Watch In Telemetry

From `motorController` JSON:
- `state`
- `lastStopReason`
- `current_mA`
- `hallDirection` and `hallConfidence`
- `endstopTriggered`

Interpretation:
- `lastStopReason = spike`: current threshold likely too low or mechanical jam
- `lastStopReason = stall`: hall pulses not detected (sensor/wiring/timing)
- `lastStopReason = endstop`: endstop wiring/polarity/config may be wrong, or it is truly at bottom

## 4. Hardware Checks (after settings)

1. **Power path**
- Verify actuator supply voltage at the driver under load.
- Confirm ground is common between ESP32, BTS7960, and motor supply.

2. **BTS7960 control wiring**
- `R_EN` and `L_EN` actually reach HIGH when running.
- `RPWM`/`LPWM` are connected to configured GPIOs.
- Motor outputs from BTS7960 to actuator are secure.

3. **Endstop wiring**
- Verify switch type and polarity match config:
  - `endstopActiveLow = true` means triggered when pin reads LOW.
- A stuck-triggered endstop can prevent expected movement direction.

4. **Hall sensor wiring**
- Confirm Hall A/B are on the configured pins and receiving valid transitions.
- If hall wiring is wrong/noisy, stall protection can stop movement quickly.

5. **Current sense wiring**
- Verify `R_IS` and `L_IS` are connected to ADC pins.
- Verify resistor-to-GND values match `isResistorOhms` (default 4.7k).
- Bad IS wiring/calibration can cause false spike stops.

## 5. Fast Isolation Test (safe and temporary)

Use this only to isolate cause, then restore safety settings:

- Increase `currentSpikeThresholdmA` significantly
- Increase `stallTimeoutMs` and `stallStartGraceMs`
- Set `pwmMin` higher (e.g. 120)
- Try hold `Up` / `Down`

If motor now moves, problem is likely threshold/timing calibration, not base drive wiring.

## 6. Restore Safe Settings

After motion is confirmed:
- Re-tune current spike threshold from measured normal vs stall current.
- Re-tighten stall timers.
- Confirm endstop stops immediately on close/down direction.
- Re-enable/confirm target distance behavior if you disabled it for testing.
