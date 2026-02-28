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

/**swerve drivetrain controller for 3-wheeled layout*/
public class Drivetrain {
    private final SwerveModule l, r, b;
    private final IMU imu;
    private final VoltageSensor vSens;
    private final Kinematics kin;

    private final Queue<Double> vQ = new LinkedList<>();
    private double vSum = 0;

    public Drivetrain(HardwareMap hw) {
        l = new SwerveModule(hw.get(DcMotorEx.class, Constants.L_DRIVE), hw.get(CRServo.class, Constants.L_STEER), hw.get(AnalogInput.class, Constants.L_ENC), Constants.L_DRIVE_REV, Constants.L_STEER_REV, Constants.L_OFF, 0);
        r = new SwerveModule(hw.get(DcMotorEx.class, Constants.R_DRIVE), hw.get(CRServo.class, Constants.R_STEER), hw.get(AnalogInput.class, Constants.R_ENC), Constants.R_DRIVE_REV, Constants.R_STEER_REV, Constants.R_OFF, 0);
        b = new SwerveModule(hw.get(DcMotorEx.class, Constants.B_DRIVE), hw.get(CRServo.class, Constants.B_STEER), hw.get(AnalogInput.class, Constants.B_ENC), Constants.B_DRIVE_REV, Constants.B_STEER_REV, Constants.B_OFF, 0);

        kin = new Kinematics();
        imu = hw.get(IMU.class, Constants.IMU);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(Constants.LOGO, Constants.USB)));
        //guard:use iterator safely and allow null if no volt data present
        java.util.Iterator<VoltageSensor> it = hw.voltageSensor.iterator();
        vSens = it.hasNext() ? it.next() : null;
    }

    public void update() {
        l.update(); r.update(); b.update();
        
        //guard:if no sensor available, use nominal voltage as fallback
        double v = (vSens != null) ? vSens.getVoltage() : Constants.NOMINAL_VOLTAGE;
        vSum += v;
        vQ.add(v);
        if (vQ.size() > Constants.VOLT_FILTER_N) {
            Double p = vQ.poll();
            if (p != null) vSum -= p;
        }
    }

    public void defense() {
        //triangular kiwi lock pattern
        l.set(new ModuleState(150, 0));
        r.set(new ModuleState(-150, 0));
        b.set(new ModuleState(-90, 0));
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
        l.set(s[0]); r.set(s[1]); b.set(s[2]);
        execute();
    }

    private void execute() {
        //avoid division by zero or infinite compensation
        double vAvg = !vQ.isEmpty() ? vSum / vQ.size() : Constants.NOMINAL_VOLTAGE;
        double vComp = (vAvg <= 0 || Double.isInfinite(vAvg) || Double.isNaN(vAvg)) ? 1.0 : Constants.NOMINAL_VOLTAGE / vAvg;
        l.execute(vComp); r.execute(vComp); b.execute(vComp);
    }

    public double getHeading() { return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); }
    public void resetYaw() { imu.resetYaw(); }
    public void stop() { l.stop(); r.stop(); b.stop(); }
    public SwerveModule getL() { return l; }
    public SwerveModule getR() { return r; }
    public SwerveModule getB() { return b; }
    public double getV() { return (vSens != null) ? vSens.getVoltage() : Constants.NOMINAL_VOLTAGE; }
}
