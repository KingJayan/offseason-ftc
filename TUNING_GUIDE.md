# TeamCode Tuning Guide

Your robot works out-of-the-box with default Config values. Tune only if behavior feels wrong.

## Quick Tuning Path

### 1. Heading Hold (Start Here)
Run `Heading Hold Tuner` OpMode. Press dpad to snap heading to cardinal direction.

**What to watch for:**
- Steady snap with no wobble → good
- Wobbles around target → lower `H_KP`
- Overshoots then oscillates → raise `H_KD`

Related config: `H_KP`, `H_KD`

### 2. Acceleration Feel (If Robot Feels Sluggish)
Run `Acceleration Tuner` OpMode. Move left stick left/right and watch ramp.

**What to watch for:**
- Ramps smoothly to max → good
- Takes too long to reach max → raise `D_ACCEL`
- Braking feels slow → raise `D_DECEL`
- Same adjustments for rotation with `R_ACCEL`, `R_DECEL`

Related config: `D_ACCEL`, `D_DECEL`, `R_ACCEL`, `R_DECEL`

### 3. Stick Sensitivity (If Aiming Is Hard)
Run `Stick Curve Tuner` OpMode. Move sticks and watch input→output on telemetry.

**Common curves:**
- `LINEAR` = direct, no curve
- `SIN` = smooth ramp (default translation)
- `QUINT` = aggressive ramp (default rotation)
- `SMOOTH`, `EXP` also available

Related config: `T_MODE`, `R_MODE`

## Config Organization

All tuning values are in `config/Config.java` grouped by section:

```
TUNE THESE FIRST    ← heading hold gains + accel limits
STICK CURVES        ← stick response modes
DRIVER BEHAVIOR     ← heading hold enable, slew, precision scale
PASSIVE ALIGNMENT   ← auto-snap to cardinal directions
STALL PROTECTION    ← disabled by default
```

## Live Tuning Workflow

1. Connect to FTC Dashboard
2. Go to `Config` tab
3. Change any `Config.java` value live
4. Hit `SAVE`
5. Change persists until you edit `Config.java` and rebuild

## Defaults (You Probably Don't Need to Change These)

- `USE_HEADING_HOLD = true` (automatic heading stabilization)
- `USE_PASSIVE_ALIGN = true` (snap to cardinal dirs when near them)
- `PASSIVE_ALIGN_DEG = 2.0` (within 2° of cardinal snaps)
- `PRECISION_SCALE = 0.4` (bumper reduces stick to 40% power)
- `SLEW = true` (smooth acceleration limiting)

## If Nothing Works

1. Check hardware: are motors spinning? IMU level?
2. Check telemetry: `SwerveOp` shows heading, module angles
3. Check `Constants.java` for correct hardware device names

## Advanced Tuning (Not Required)

- Pedro Pathing: see `Tuning` OpMode (in `pedroPathing` group)
- Motor encoders: see `SwerveOffsetTuner` (shows motor pos/vel)

