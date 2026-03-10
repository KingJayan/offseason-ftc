package org.firstinspires.ftc.teamcode.TeleOp.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.helpers.util.MathUtil;

/**motor encoder readout for all three drive motors*/
@TeleOp(name="swerve offset tuner")
public class SwerveOffsetTuner extends OpMode {
    private DcMotorEx l;
    private DcMotorEx r;
    private DcMotorEx b;
    private AnalogInput lEnc;
    private AnalogInput rEnc;
    private AnalogInput bEnc;

    @Override
    public void init() {
        l = hardwareMap.get(DcMotorEx.class, Constants.L_DRIVE);
        r = hardwareMap.get(DcMotorEx.class, Constants.R_DRIVE);
        b = hardwareMap.get(DcMotorEx.class, Constants.B_DRIVE);
        lEnc = hardwareMap.get(AnalogInput.class, Constants.L_STEER_ENC);
        rEnc = hardwareMap.get(AnalogInput.class, Constants.R_STEER_ENC);
        bEnc = hardwareMap.get(AnalogInput.class, Constants.B_STEER_ENC);
    }

    @Override
    public void loop() {
        telemetry.addLine("point all wheels fwd, then copy deg into *_OFF_DEG");
        show("l", l, lEnc, Constants.L_OFF_DEG);
        show("r", r, rEnc, Constants.R_OFF_DEG);
        show("b", b, bEnc, Constants.B_OFF_DEG);
        telemetry.update();
    }

    private void show(String n, DcMotorEx m, AnalogInput enc, double offDeg) {
        double maxV = Math.max(1e-6, Constants.STEER_ANALOG_MAX_V);
        double v = Math.max(0.0, Math.min(maxV, enc.getVoltage()));
        double deg = MathUtil.wrap((v / maxV) * 360.0 - offDeg);
        telemetry.addData(n, "deg:%.1f rawv:%.3f pos:%d vel:%.2f", deg, v, m.getCurrentPosition(), m.getVelocity());
    }
}
