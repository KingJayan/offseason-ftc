package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.helpers.util.MathUtil;

/**swerve module control*/
public class SwerveModule {
    private final DcMotorEx drive;
    private final CRServo steer;
    private final AnalogInput steerEnc;

    private double curDeg = 0;
    private double tgtDeg = 0;
    private double drivePct = 0;
    private final boolean dRev, sRev;
    private final double offDeg;

    public SwerveModule(DcMotorEx drive, CRServo steer, AnalogInput steerEnc, boolean dRev, boolean sRev, double offDeg) {
        this.drive = drive;
        this.steer = steer;
        this.steerEnc = steerEnc;
        this.dRev = dRev;
        this.sRev = sRev;
        this.offDeg = offDeg;
        this.drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**axon max tracks own pos, cur assumes tgt*/
    public void update() {
        curDeg = readSteerDeg();
    }

    public void set(ModuleState state) {
        ModuleState opt = state.optimize(curDeg);
        this.tgtDeg = opt.angle;
        this.drivePct = Math.abs(state.speed) < Constants.MODULE_DB ? 0 : opt.speed;
    }

    /**send pos to axon*/
    public void execute(double vComp) {
        double err = MathUtil.wrap(tgtDeg - curDeg);
        double steerOut = Math.abs(err) <= Constants.STEER_JITTER_DEG ? 0.0 : (err * Constants.STEER_KP);
        steerOut = Math.max(-Constants.STEER_MAX_PWR, Math.min(Constants.STEER_MAX_PWR, steerOut));
        steer.setPower(sRev ? -steerOut : steerOut);

        double outPct = (dRev ? -drivePct : drivePct) * vComp;
        drive.setPower(Math.max(-1.0, Math.min(1.0, outPct)));
    }

    private double readSteerDeg() {
        double maxV = Math.max(1e-6, Constants.STEER_ANALOG_MAX_V);
        double v = Math.max(0.0, Math.min(maxV, steerEnc.getVoltage()));
        double rawDeg = (v / maxV) * 360.0;
        return MathUtil.wrap(rawDeg - offDeg);
    }

    public double getCurDeg() { return curDeg; }
    public double getTgtDeg() { return tgtDeg; }
    public double getDrivePos() { return drive.getCurrentPosition(); }
    public double getDriveVel() { return drive.getVelocity(); }
    public void resetDrive() {
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void stop() {
        steer.setPower(0);
        drive.setPower(0);
    }
}
