package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SwerveModule {
    private final DcMotorEx drive;
    private final CRServo steer;
    private final AnalogInput sensor;

    //pid constants (loaded from SwerveConstants)
    /// TODO: TUNE
    private double kP;
    private double kD;
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
    private double lastError = 0.0;
    private final ElapsedTime timer = new ElapsedTime();

    //direction and offset config
    private final boolean driveReversed;
    private final boolean steerReversed;
    private final double angleOffset;

    public SwerveModule(DcMotorEx drive, CRServo steer, AnalogInput sensor,
                        boolean driveReversed, boolean steerReversed, double angleOffset) {
        this.drive = drive;
        this.steer = steer;
        this.sensor = sensor;
        this.driveReversed = driveReversed;
        this.steerReversed = steerReversed;
        this.angleOffset = angleOffset;

        // load constants
        this.kP = SwerveConstants.STEER_KP;
        this.kD = SwerveConstants.STEER_KD;
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
            currentModuleAngle = currentServoAngle / SwerveConstants.GEAR_RATIO;
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
        double raw = (voltage / SwerveConstants.MAX_SENSOR_VOLTAGE) * 360.0;
        return MathUtils.angleWrap(raw - angleOffset);
    }

    /**
     * set target state for module using SwerveModuleState
     */
    public void setTargetState(SwerveModuleState state) {
        SwerveModuleState optimized = state.optimize(currentModuleAngle);
        this.targetAngle = optimized.angle;
        this.drivePower = optimized.speed;
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
        double dt = timer.seconds();
        timer.reset();

        double error = MathUtils.angleWrap(targetAngle - currentModuleAngle);
        double derivative = (dt > 0) ? (error - lastError) / dt : 0.0;
        lastError = error;

        double steerPower = 0.0;
        if (Math.abs(error) > angleTolerance) {
            steerPower = kP * error + kD * derivative;

            // static compensation
            if (steerPower > 0) {
                steerPower += kStatic;
            } else if (steerPower < 0) {
                steerPower -= kStatic;
            }

            // clamp to valid range
            steerPower = Math.max(-1.0, Math.min(1.0, steerPower));
        }

        steer.setPower(steerReversed ? -steerPower : steerPower);

        //drive motor
        double finalDrivePower = drivePower;
        if (driveReversed) {
            finalDrivePower = -finalDrivePower;
        }

        finalDrivePower *= voltageCompensation;
        finalDrivePower = Math.max(-1.0, Math.min(1.0, finalDrivePower));

        drive.setPower(finalDrivePower);
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
        lastError = 0.0;
    }

    public void reloadConstants() {
        this.kP = SwerveConstants.STEER_KP;
        this.kD = SwerveConstants.STEER_KD;
        this.kStatic = SwerveConstants.STEER_KSTATIC;
        this.angleTolerance = SwerveConstants.STEER_TOLERANCE;
    }
}
