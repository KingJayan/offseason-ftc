package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SwerveModule {
    private final DcMotorEx drive;
    private final CRServo steer;
    private final AnalogInput sensor;

    //pid constants (loaded from SwerveConstants)
    /// TODO: TUNE
    private double kP;
    private double kStatic;
    private double angleTolerance;

    //odom tracking for rollover
    private double lastServoAngle = 0.0;
    private double totalServoRotations = 0.0;
    private boolean initialized = false;

    // curr state
    private double currentModuleAngle = 0.0;
    private double targetAngle = 0.0;
    private double drivePower = 0.0;
    private boolean driveReversed = false;

    public SwerveModule(DcMotorEx drive, CRServo steer, AnalogInput sensor) {
        this.drive = drive;
        this.steer = steer;
        this.sensor = sensor;

        // load constants
        this.kP = SwerveConstants.STEER_KP;
        this.kStatic = SwerveConstants.STEER_KSTATIC;
        this.angleTolerance = SwerveConstants.STEER_TOLERANCE;

        // configure drive motor
        this.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * track cumulative servo rotations to get abs angle
     */
    public void update() {
        double currentServoAngle = getServoAngle();

        if (!initialized) {
            lastServoAngle = currentServoAngle;
            initialized = true;
            return;
        }

        double delta = currentServoAngle - lastServoAngle;

        if (delta > 180.0) {
            totalServoRotations -= 1.0; //low-high
        } else if (delta < -180.0) {

            totalServoRotations += 1.0;//high-low
        }

        lastServoAngle = currentServoAngle;

        //calc module angle: total servo travel/ gear ratio
        double totalServoDegrees = (totalServoRotations * 360.0) + currentServoAngle;
        currentModuleAngle = totalServoDegrees / SwerveConstants.GEAR_RATIO;
    }

    /** read raw server angle
     */
    private double getServoAngle() {
        double voltage = sensor.getVoltage();
        voltage = Math.min(SwerveConstants.MAX_SENSOR_VOLTAGE, Math.max(0, voltage));
        return (voltage / SwerveConstants.MAX_SENSOR_VOLTAGE) * 360.0;
    }

    /**
     * set target state for module using SwerveModuleState
     */
    public void setTargetState(SwerveModuleState state) {
        SwerveModuleState optimized = state.optimize(currentModuleAngle);
        this.targetAngle = optimized.angle;
        this.drivePower = optimized.speed;
        this.driveReversed = (optimized.speed != state.speed);
    }

    /**
     * set target state for module, optimiz
     */
    public void setTargetState(double angle, double power) {
        setTargetState(new SwerveModuleState(angle, power));
    }

    /**
     * exec control loop
     */
    public void execute(double voltageCompensation) {

        double error = angleWrap(targetAngle - currentModuleAngle);

        double steerPower = 0.0;
        if (Math.abs(error) > angleTolerance) {
            steerPower = kP * error;

            // static compensation
            if (steerPower > 0) {
                steerPower += kStatic;
            } else if (steerPower < 0) {
                steerPower -= kStatic;
            }

            // clamp to valid range
            steerPower = Math.max(-1.0, Math.min(1.0, steerPower));
        }

        steer.setPower(steerPower);

        //drive motor
        double finalDrivePower = drivePower;
        if (driveReversed) {
            finalDrivePower = -finalDrivePower;
        }

        finalDrivePower *= voltageCompensation;
        finalDrivePower = Math.max(-1.0, Math.min(1.0, finalDrivePower));

        drive.setPower(finalDrivePower);
    }

    private double angleWrap(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }


    public double getCurrentAngle() {
        return currentModuleAngle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void stop() {
        drive.setPower(0);
        steer.setPower(0);
    }

    //tune settiers
    public void setKp(double kP) { this.kP = kP; }
    public void setKStatic(double kStatic) { this.kStatic = kStatic; }
    public void setAngleTolerance(double tolerance) { this.angleTolerance = tolerance; }

    //reload constants from SwerveConstants
    public void reloadConstants() {
        this.kP = SwerveConstants.STEER_KP;
        this.kStatic = SwerveConstants.STEER_KSTATIC;
        this.angleTolerance = SwerveConstants.STEER_TOLERANCE;
    }
}
