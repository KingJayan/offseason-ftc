package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.Swerve.Kinematics;
import org.firstinspires.ftc.teamcode.Swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.Swerve.ModuleState;

/**swerve drivetrain for pedro pathing*/
public class SwerveDrivetrain extends Drivetrain {
    private final SwerveModule lf, rf, lb, rb;
    private final VoltageSensor vSens;
    private final Kinematics kin;
    private final double[] o = new double[8];
    private double xV = 0, yV = 0;

    public SwerveDrivetrain(HardwareMap hw) {
        lf = new SwerveModule(hw.get(DcMotorEx.class, Constants.LF_DRIVE), hw.get(CRServo.class, Constants.LF_STEER), hw.get(AnalogInput.class, Constants.LF_ENC), Constants.LF_DRIVE_REV, Constants.LF_STEER_REV, Constants.LF_OFF);
        rf = new SwerveModule(hw.get(DcMotorEx.class, Constants.RF_DRIVE), hw.get(CRServo.class, Constants.RF_STEER), hw.get(AnalogInput.class, Constants.RF_ENC), Constants.RF_DRIVE_REV, Constants.RF_STEER_REV, Constants.RF_OFF);
        lb = new SwerveModule(hw.get(DcMotorEx.class, Constants.LB_DRIVE), hw.get(CRServo.class, Constants.LB_STEER), hw.get(AnalogInput.class, Constants.LB_ENC), Constants.LB_DRIVE_REV, Constants.LB_STEER_REV, Constants.LB_OFF);
        rb = new SwerveModule(hw.get(DcMotorEx.class, Constants.RB_DRIVE), hw.get(CRServo.class, Constants.RB_STEER), hw.get(AnalogInput.class, Constants.RB_ENC), Constants.RB_DRIVE_REV, Constants.RB_STEER_REV, Constants.RB_OFF);

        kin = new Kinematics();
        vSens = hw.voltageSensor.iterator().next();
        setNominalVoltage(Constants.NOMINAL_VOLTAGE);
    }

    public void update() {
        lf.update(); rf.update(); lb.update(); rb.update();
    }

    @Override
    public double[] calculateDrive(Vector corr, Vector head, Vector cent, double h) {
        //pedro x is fwd, y is lft. kin x is rgt, y is fwd.
        double f = corr.getXComponent() + cent.getXComponent();
        double l = corr.getYComponent() + cent.getYComponent();
        double rx = head.getXComponent();

        ModuleState[] s = kin.calculate(-l, f, rx);
        o[0] = s[0].angle; o[1] = s[0].speed;
        o[2] = s[1].angle; o[3] = s[1].speed;
        o[4] = s[2].angle; o[5] = s[2].speed;
        o[6] = s[3].angle; o[7] = s[3].speed;
        return o;
    }

    @Override
    public void runDrive(double[] outputs) {
        update();
        if (Math.abs(outputs[1]) + Math.abs(outputs[3]) + Math.abs(outputs[5]) + Math.abs(outputs[7]) < Constants.MODULE_DB) {
            stop(); return;
        }
        lf.set(new ModuleState(outputs[0], outputs[1]));
        rf.set(new ModuleState(outputs[2], outputs[3]));
        lb.set(new ModuleState(outputs[4], outputs[5]));
        rb.set(new ModuleState(outputs[6], outputs[7]));

        double v = isVoltageCompensation() ? getNominalVoltage() / getVoltage() : 1.0;
        lf.execute(v); rf.execute(v); lb.execute(v); rb.execute(v);
    }

    @Override public void updateConstants() {}
    @Override public void breakFollowing() { stop(); }
    @Override public void startTeleopDrive() {}
    @Override public void startTeleopDrive(boolean fcd) {}
    @Override public double xVelocity() { return xV; }
    @Override public double yVelocity() { return yV; }
    @Override public void setXVelocity(double v) { xV = v; }
    @Override public void setYVelocity(double v) { yV = v; }
    @Override public double getVoltage() { return vSens.getVoltage(); }
    @Override public String debugString() { return String.format("lf:%.1f rf:%.1f lb:%.1f rb:%.1f", lf.getCurA(), rf.getCurA(), lb.getCurA(), rb.getCurA()); }

    public void stop() {
        lf.stop(); rf.stop(); lb.stop(); rb.stop();
    }
}
