package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.helpers.util.MathUtil;

/**swerve module control*/
public class SwerveModule {
    private final DcMotorEx drive;
    private final Servo steer;

    private double curDeg = 0;
    private double tgtDeg = 0;
    private double drivePct = 0;
    private final boolean dRev, sRev;
    private final double offDeg;

    public SwerveModule(DcMotorEx drive, Servo steer, boolean dRev, boolean sRev, double offDeg) {
        this.drive = drive;
        this.steer = steer;
        this.dRev = dRev;
        this.sRev = sRev;
        this.offDeg = offDeg;
        this.drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**axon max tracks own pos, cur assumes tgt*/
    public void update() {
        curDeg = tgtDeg;
    }

    public void set(ModuleState state) {
        ModuleState opt = state.optimize(curDeg);
        this.tgtDeg = opt.angle;
        this.drivePct = Math.abs(state.speed) < Constants.MODULE_DB ? 0 : opt.speed;
    }

    /**send pos to axon*/
    public void execute(double vComp) {
        //map -180 to 180 to 0-1 range for servo
        double pos = MathUtil.wrap(tgtDeg + offDeg);
        double servoPos = (pos + 180.0) / 360.0;
        steer.setPosition(sRev ? 1.0 - servoPos : servoPos);

        double outPct = (dRev ? -drivePct : drivePct) * vComp;
        drive.setPower(Math.max(-1.0, Math.min(1.0, outPct)));
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
        drive.setPower(0);
    }
}
