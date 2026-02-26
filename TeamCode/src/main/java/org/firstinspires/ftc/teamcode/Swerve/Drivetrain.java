package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.Constants;

import java.util.LinkedList;
import java.util.Queue;

/**swerve drivetrain controller*/
public class Drivetrain {
    private final SwerveModule lf, rf, lb, rb;
    private final IMU imu;
    private final VoltageSensor vSens;
    private final Kinematics kin;
    
    private final Queue<Double> vQ = new LinkedList<>();
    private double vSum = 0;

    public Drivetrain(HardwareMap hw) {
        lf = new SwerveModule(hw.get(DcMotorEx.class, Constants.LF_DRIVE), hw.get(CRServo.class, Constants.LF_STEER), hw.get(AnalogInput.class, Constants.LF_ENC), Constants.LF_DRIVE_REV, Constants.LF_STEER_REV, Constants.LF_OFF, 0);
        rf = new SwerveModule(hw.get(DcMotorEx.class, Constants.RF_DRIVE), hw.get(CRServo.class, Constants.RF_STEER), hw.get(AnalogInput.class, Constants.RF_ENC), Constants.RF_DRIVE_REV, Constants.RF_STEER_REV, Constants.RF_OFF, 0);
        lb = new SwerveModule(hw.get(DcMotorEx.class, Constants.LB_DRIVE), hw.get(CRServo.class, Constants.LB_STEER), hw.get(AnalogInput.class, Constants.LB_ENC), Constants.LB_DRIVE_REV, Constants.LB_STEER_REV, Constants.LB_OFF, 0);
        rb = new SwerveModule(hw.get(DcMotorEx.class, Constants.RB_DRIVE), hw.get(CRServo.class, Constants.RB_STEER), hw.get(AnalogInput.class, Constants.RB_ENC), Constants.RB_DRIVE_REV, Constants.RB_STEER_REV, Constants.RB_OFF, 0);

        kin = new Kinematics();
        imu = hw.get(IMU.class, Constants.IMU);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(Constants.LOGO, Constants.USB)));
        vSens = hw.voltageSensor.iterator().next();
    }

    public void update() {
        lf.update(); rf.update(); lb.update(); rb.update();
        
        //vcomp filter
        double v = vSens.getVoltage();
        vSum += v;
        vQ.add(v);
        if (vQ.size() > Constants.VOLT_FILTER_N) vSum -= vQ.poll();
    }

    public void defense() {
        lf.set(new ModuleState(45, 0));
        rf.set(new ModuleState(-45, 0));
        lb.set(new ModuleState(-45, 0));
        rb.set(new ModuleState(45, 0));
        execute();
    }

    public void drive(double x, double y, double rx, boolean fcd) {
        if (fcd) {
            double h = Math.toRadians(getHeading());
            double[] v = Kinematics.fcd(x, y, h);
            x = v[0]; y = v[1];
        }
        if (Math.abs(x) < Constants.STICK_DB && Math.abs(y) < Constants.STICK_DB && Math.abs(rx) < Constants.STICK_DB) {
            stop(); return;
        }
        ModuleState[] s = kin.calculate(x, y, rx);
        lf.set(s[0]); rf.set(s[1]); lb.set(s[2]); rb.set(s[3]);
        execute();
    }

    private void execute() {
        double vAvg = vSum / Math.max(1, vQ.size());
        double vComp = Constants.NOMINAL_VOLTAGE / vAvg;
        lf.execute(vComp); rf.execute(vComp); lb.execute(vComp); rb.execute(vComp);
    }

    public double getHeading() { return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); }
    public void resetYaw() { imu.resetYaw(); }
    public void stop() { lf.stop(); rf.stop(); lb.stop(); rb.stop(); }
    public SwerveModule getLF() { return lf; }
    public SwerveModule getRF() { return rf; }
    public SwerveModule getLB() { return lb; }
    public SwerveModule getRB() { return rb; }
    public double getV() { return vSens.getVoltage(); }
}
