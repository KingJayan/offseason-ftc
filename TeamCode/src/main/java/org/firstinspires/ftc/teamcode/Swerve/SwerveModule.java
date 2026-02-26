package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Config;
import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.helpers.util.MathUtil;

/**swerve module control*/
public class SwerveModule {
    private final DcMotorEx drive;
    private final CRServo steer;
    private final AnalogInput enc;

    private double curA = 0;
    private double tgtA = 0;
    private double pwr = 0;
    private double lastM = 0;
    private double lastS = 0;
    private double rots = 0;
    private boolean init = false;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime stallTimer = new ElapsedTime();
    private double lastStallA = 0;
    private boolean stalled = false;

    private final boolean dRev, sRev;
    private final double off;

    public SwerveModule(DcMotorEx drive, CRServo steer, AnalogInput enc, boolean dRev, boolean sRev, double off) {
        this.drive = drive;
        this.steer = steer;
        this.enc = enc;
        this.dRev = dRev;
        this.sRev = sRev;
        this.off = off;
        this.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**track rotations for rollover*/
    public void update() {
        double curS = getS();
        if (!init) {
            lastS = curS;
            curA = curS / Constants.GEAR_RATIO;
            lastM = curA;
            lastStallA = curA;
            init = true;
            return;
        }
        double delta = curS - lastS;
        if (delta > 180.0) rots--;
        else if (delta < -180.0) rots++;
        lastS = curS;
        curA = ((rots * 360.0) + curS) / Constants.GEAR_RATIO;

        //stall check logic
        if (Config.USE_STALL_PROT) {
            if (Math.abs(curA - lastStallA) > Config.STALL_THRESHOLD) {
                lastStallA = curA;
                stallTimer.reset();
                stalled = false;
            } else if (stallTimer.seconds() > Config.STALL_TIMEOUT && Math.abs(tgtA - curA) > Constants.TOLERANCE) {
                stalled = true;
            }
        } else {
            stalled = false;
        }
    }

    private double getS() {
        double v = Math.min(Constants.MAX_V, Math.max(0, enc.getVoltage()));
        return MathUtil.wrap((v / Constants.MAX_V) * 360 - off);
    }

    public void set(ModuleState state) {
        if (Math.abs(state.speed) < Constants.MODULE_DB) {
            this.pwr = 0;
            return;
        }
        ModuleState opt = state.optimize(curA);
        this.tgtA = opt.angle;
        this.pwr = opt.speed;
    }

    /**pid loop*/
    public void execute(double vComp) {
        double dt = timer.seconds();
        timer.reset();
        double err = MathUtil.wrap(tgtA - curA);
        double d = dt > 0 ? (curA - lastM) / dt : 0;
        lastM = curA;

        double sPwr = 0;
        if (Math.abs(err) > Constants.TOLERANCE && !stalled) {
            sPwr = Constants.KP * err - Constants.KD * d;
            sPwr += Math.signum(sPwr) * Constants.KSTATIC;
            sPwr = Math.max(-1.0, Math.min(1.0, sPwr));
        }
        steer.setPower(sRev ? -sPwr : sPwr);

        double dPwr = (dRev ? -pwr : pwr) * vComp;
        drive.setPower(Math.max(-1.0, Math.min(1.0, dPwr)));
    }

    public double getCurA() { return curA; }
    public double getTgtA() { return tgtA; }
    public boolean isStalled() { return stalled; }
    public void stop() {
        drive.setPower(0);
        steer.setPower(0);
        lastM = curA;
    }
}
