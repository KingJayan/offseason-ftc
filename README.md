# swerve-ftc

this repo runs a 3-module differential swerve drivetrain with teleop + pedro pathing support

## teamcode overview

- main teleop: `teamcode/TeleOp/SwerveOp.java`
- basic auto path check: `teamcode/Autonomous/BasicAuto.java`
- drivetrain core: `teamcode/Swerve/Drivetrain.java`
- module control: `teamcode/Swerve/SwerveModule.java`
- module math: `teamcode/Swerve/Kinematics.java`

## controls

- `ps` toggles field centric / robot centric
- `dpad` snaps to cardinals
- `x` resets yaw
- `b` holds defense pose while pressed
- right stick x overrides heading hold while you command rotation
- bumpers scale translation + rotation for precision driving

## hardware assumptions

- steering uses continuous rotation servo with analog wire
- drive encoders are plugged into motor ports

<!--## setup

1. configure all servos as `CRServo`
2. plug each axon analog wire into an analog input port
3. run `swerve offset tuner`
4. point all three wheels straight forward by hand
5. copy the shown `deg` values into `L_OFF_DEG`, `R_OFF_DEG`, `B_OFF_DEG`
6. if a module turns the wrong way, flip that module's `*_STEER_REV`
7. if a drive wheel spins backward, flip that module's `*_DRIVE_REV`
8. tune `STEER_KP` and `STEER_MAX_PWR` only after offsets and directions are correct
9. run `Heading Hold Tuner` and tune `H_KP`, `H_KD`, then `H_KI` in very small steps


### heading hold quick tune

- `H_KP` = main correction strength
- `H_KD` = dampens imu noise/overshoot through filtered derivative
- `H_KI` = fixes steady bias/drift; start at `0.0` and increase slowly
- `H_I_ZONE_DEG` limits where integral can build
- `H_I_MAX` caps integral buildup (antiwindup)
- `MODE_TOGGLE_DB_SEC`, `SNAP_DB_SEC`, `YAW_RESET_DB_SEC` debounce controller toggles

### configs

- `config/Constants.java` = hardware names, geometry, deadbands, steer hardware tuning
- `config/Config.java` = runtime tuning knobs and driver prefs

-->