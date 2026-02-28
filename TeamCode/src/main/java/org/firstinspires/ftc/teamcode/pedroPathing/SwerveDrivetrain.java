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

/**swerve drivetrain for pedro pathing (3-wheeled)*/
public class SwerveDrivetrain extends Drivetrain {
    private final SwerveModule l, r, b;
    private final VoltageSensor vSens;
    private final Kinematics kin;
    private final double[] o = new double[6];
    private double xV = 0, yV = 0;

    public SwerveDrivetrain(HardwareMap hw) {
        l = new SwerveModule(hw.get(DcMotorEx.class, Constants.L_DRIVE), hw.get(CRServo.class, Constants.L_STEER), hw.get(AnalogInput.class, Constants.L_ENC), Constants.L_DRIVE_REV, Constants.L_STEER_REV, Constants.L_OFF, 0);
        r = new SwerveModule(hw.get(DcMotorEx.class, Constants.R_DRIVE), hw.get(CRServo.class, Constants.R_STEER), hw.get(AnalogInput.class, Constants.R_ENC), Constants.R_DRIVE_REV, Constants.R_STEER_REV, Constants.R_OFF, 0);
        b = new SwerveModule(hw.get(DcMotorEx.class, Constants.B_DRIVE), hw.get(CRServo.class, Constants.B_STEER), hw.get(AnalogInput.class, Constants.B_ENC), Constants.B_DRIVE_REV, Constants.B_STEER_REV, Constants.B_OFF, 0);

        kin = new Kinematics();
        java.util.Iterator<VoltageSensor> it = hw.voltageSensor.iterator();
        vSens = it.hasNext() ? it.next() : null;
        setNominalVoltage(Constants.NOMINAL_VOLTAGE);
    }

    public void update() {
        l.update(); r.update(); b.update();
    }

    @Override
    public double[] calculateDrive(Vector corr, Vector head, Vector cent, double h) {
        double f = corr.getXComponent() + cent.getXComponent();
        double lft = corr.getYComponent() + cent.getYComponent();
        double rx = head.getXComponent();

        ModuleState[] s = kin.calculate(-lft, f, rx);
        o[0] = s[0].angle; o[1] = s[0].speed;
        o[2] = s[1].angle; o[3] = s[1].speed;
        o[4] = s[2].angle; o[5] = s[2].speed;
        return o;
    }


    @Override
    public void runDrive(double[] outputs) {
        update();
        if (Math.abs(outputs[1]) + Math.abs(outputs[3]) + Math.abs(outputs[5]) < Constants.MODULE_DB) {
            stop(); return;
        }
        l.set(new ModuleState(outputs[0], outputs[1]));
        r.set(new ModuleState(outputs[2], outputs[3]));
        b.set(new ModuleState(outputs[4], outputs[5]));

        double v = isVoltageCompensation() ? getNominalVoltage() / Math.max(0.001, getVoltage()) : 1.0;
        l.execute(v); r.execute(v); b.execute(v);
    }

    @Override public void updateConstants() {}
    @Override public void breakFollowing() { stop(); }
    @Override public void startTeleopDrive() {}
    @Override public void startTeleopDrive(boolean fcd) {}
    @Override public double xVelocity() { return xV; }
    @Override public double yVelocity() { return yV; }
    @Override public void setXVelocity(double v) { xV = v; }
    @Override public void setYVelocity(double v) { yV = v; }
    @Override public double getVoltage() { return (vSens != null) ? vSens.getVoltage() : Constants.NOMINAL_VOLTAGE; }
    @Override public String debugString() { return String.format("l:%.1f r:%.1f b:%.1f", l.getCurDeg(), r.getCurDeg(), b.getCurDeg()); }

    public void stop() {
        l.stop(); r.stop(); b.stop();
    }
}
