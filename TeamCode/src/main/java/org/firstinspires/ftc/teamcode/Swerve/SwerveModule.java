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
    private double lastServoAngle = 0;
    private double totalRots = 0;
    private boolean init = false;

    // curr state
    private double currModuleAngle = 0;
    private double targetAngle = 0;
    private double drivePower = 0;
    private double lastError = 0;
    private final ElapsedTime timer = new ElapsedTime();

    //direction and offset config
    private final boolean dReversed;
    private final boolean sReversed;
    private final double angleOffset;

    public SwerveModule(DcMotorEx drive, CRServo steer, AnalogInput sensor,
                        boolean dReversed, boolean sReversed, double angleOffset) {
        this.drive = drive;
        this.steer = steer;
        this.sensor = sensor;
        this.dReversed = dReversed;
        this.sReversed = sReversed;
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
        double currServoAngle = getServoAngle();

        if (!init) {
            lastServoAngle = currServoAngle;
            currModuleAngle = currServoAngle /SwerveConstants.GEAR_RATIO;
            init = true;
            return;
        }

        double delta = currServoAngle-lastServoAngle;

        if (delta >180.0) {
            totalRots -= 1.0; //low-high
        } else if (delta< -180.0) {

            totalRots += 1.0;//high-low
        }

        lastServoAngle=currServoAngle;

        //calc module angle: total servo travel/ gear ratio
        double sumServoDeg = (totalRots * 360.0) +currServoAngle;
        currModuleAngle = sumServoDeg / SwerveConstants.GEAR_RATIO;
    }

    /** read raw server angle
     */
    private double getServoAngle() {
        double voltage = sensor.getVoltage();
        voltage = Math.min(SwerveConstants.MAX_SENSOR_VOLTAGE, Math.max(0, voltage));
        double raw = (voltage / SwerveConstants.MAX_SENSOR_VOLTAGE) *360;
        return MathUtils.angleWrap(raw-angleOffset);
    }

    /**
     * set target state for module using SwerveModuleState
     */
    public void setTargetState(SwerveModuleState state) {
        SwerveModuleState optimized = state.optimize(currModuleAngle);
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

        double error = MathUtils.angleWrap(targetAngle - currModuleAngle);
        double d= (dt > 0) ? (error - lastError) /dt : 0.0;
        lastError = error;

        double steerPower = 0.0;
        if (Math.abs(error) > angleTolerance) {
            steerPower = kP*error + kD*d;

            // static compensation
            if (steerPower > 0) {
                steerPower += kStatic;
            } else if (steerPower<0) {
                steerPower -= kStatic;
            }

            // clamp to valid range
            steerPower = Math.max(-1.0, Math.min(1.0, steerPower));
        }

        steer.setPower(sReversed ? -steerPower : steerPower);

        //drive motor
        double plegit = drivePower;
        if (dReversed) {
            plegit = -plegit;
        }

        plegit *= voltageCompensation;
        plegit = Math.max(-1.0, Math.min(1.0, plegit));

        drive.setPower(plegit);
    }


    public double getCurrAngle() {
        return currModuleAngle;
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
