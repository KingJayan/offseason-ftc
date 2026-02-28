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

    private double curDeg = 0;
    private double tgtDeg = 0;
    private double drivePct = 0;
    private double lastDeg = 0;
    private double lastServoDeg = 0;
    private double totalRots = 0;
    private boolean init = false;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime stallTimer = new ElapsedTime();
    private double lastStallDeg = 0;
    private boolean stalled = false;

    private final boolean dRev, sRev;
    private final double offsetDeg;

    public SwerveModule(DcMotorEx drive, CRServo steer, AnalogInput enc, boolean dRev, boolean sRev, double offsetDeg, double initRots) {
        this.drive = drive;
        this.steer = steer;
        this.enc = enc;
        this.dRev = dRev;
        this.sRev = sRev;
        this.offsetDeg = offsetDeg;
        this.totalRots = initRots;
        this.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**track rotations for rollover*/
    public void update() {
        double curS = getServoDeg();
        if (!init) {
            lastServoDeg = curS;
            curDeg = ((totalRots * 360.0) + curS) / Constants.GEAR_RATIO;
            lastDeg = curDeg;
            lastStallDeg = curDeg;
            init = true;
            return;
        }
        double delta = curS - lastServoDeg;
        if (delta > 180.0) totalRots--;
        else if (delta < -180.0) totalRots++;
        lastServoDeg = curS;
        curDeg = ((totalRots * 360.0) + curS) / Constants.GEAR_RATIO;

        if (Config.USE_STALL_PROT) {
            if (Math.abs(curDeg - lastStallDeg) > Config.STALL_THRESHOLD) {
                lastStallDeg = curDeg;
                stallTimer.reset();
                stalled = false;
            } else if (stallTimer.seconds() > Config.STALL_TIMEOUT && Math.abs(tgtDeg - curDeg) > Constants.TOLERANCE) {
                stalled = true;
            }
        } else {
            stalled = false;
        }
    }

    private double getServoDeg() {
        double v = Math.min(Constants.MAX_V, Math.max(0, enc.getVoltage()));
        return MathUtil.wrap((v / Constants.MAX_V) * 360 - offsetDeg);
    }

    public void set(ModuleState state) {
        //allow angle updates even if speed is 0 for defense
        ModuleState opt = state.optimize(curDeg);
        this.tgtDeg = opt.angle;
        this.drivePct = Math.abs(state.speed) < Constants.MODULE_DB ? 0 : opt.speed;
    }

    /**pid loop*/
    public void execute(double vComp) {
        double dtSec = timer.seconds();
        timer.reset();
        double errDeg = MathUtil.wrap(tgtDeg - curDeg);
        double dDegSec = dtSec > 0 ? (curDeg - lastDeg) / dtSec : 0;
        lastDeg = curDeg;

        double sPwr = 0;
        if (Math.abs(errDeg) > Constants.TOLERANCE && !stalled) {
            sPwr = Constants.KP * errDeg - Constants.KD * dDegSec;
            sPwr += Math.signum(sPwr) * Constants.KSTATIC;
            sPwr = Math.max(-1.0, Math.min(1.0, sPwr));
        }
        
        // suppression if parked and aligned
        if (Math.abs(drivePct) < 0.01 && Math.abs(errDeg) < Constants.STEER_JITTER_DEG) sPwr = 0;

        steer.setPower(sRev ? -sPwr : sPwr);

        double outPct = (dRev ? -drivePct : drivePct) * vComp;
        drive.setPower(Math.max(-1.0, Math.min(1.0, outPct)));
    }

    public double getCurDeg() { return curDeg; }
    public double getTgtDeg() { return tgtDeg; }
    public double getRots() { return totalRots; }
    public boolean isStalled() { return stalled; }
    public void stop() {
        drive.setPower(0);
        steer.setPower(0);
        lastDeg = curDeg;
    }
}
