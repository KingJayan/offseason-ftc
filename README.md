# swerve-ftc

this repo runs a 3-module differential swerve drivetrain with teleop + pedro pathing support

## teamcode overview

- main teleop: `teamcode/TeleOp/SwerveOp.java`
- basic auto path check: `teamcode/Autonomous/BasicAuto.java`
- drivetrain core: `teamcode/Swerve/Drivetrain.java`
- module math: `teamcode/Swerve/Kinematics.java`
- pedro wrapper drivetrain: `teamcode/pedroPathing/SwerveDrivetrain.java`
<!--
## controls

current `SwerveOp` behavior:
- ps toggles field centric
- dpad press snaps heading once per press
- x resets yaw once per press
- b - defense pose while held
- right stick x manual rotate input cancels rx hold until stick returns to neutral
-->
## hardware assumptions

- separate analog steering encoder object NOT supported (ex rev through bore)
- pinpoint + dead wheels or just motor encoders are supoported

## configs

- `config/Constants.java` = hardware names, geometry, deadbands, fixed values
- `config/Config.java` = runtime tuning knobs and driver pref

## tests

- math tests: `src/test/.../teamcode/Swerve/SwerveMathTest.java`
- drivetrain tests: `src/test/.../teamcode/Swerve/DrivetrainTest.java`
