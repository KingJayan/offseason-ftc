package org.firstinspires.ftc.teamcode.TeleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.config.Constants;

/**
 * finds absolute encoder offsets for 3-module layout
 */
@TeleOp(name="swerve offset tuner")
public class SwerveOffsetTuner extends OpMode {
    private AnalogInput l, r, b;

    @Override
    public void init() {
        l = hardwareMap.get(AnalogInput.class, Constants.L_ENC);
        r = hardwareMap.get(AnalogInput.class, Constants.R_ENC);
        b = hardwareMap.get(AnalogInput.class, Constants.B_ENC);
    }

    @Override
    public void loop() {
        telemetry.addLine("align all wheels forward and record:");
        showRaw("L", l);
        showRaw("R", r);
        showRaw("B", b);
        telemetry.update();
    }

    private void showRaw(String n, AnalogInput s) {
        double v = s.getVoltage();
        double a = (v / Constants.MAX_V) * 360.0;
        telemetry.addData(n, "raw: %.2f (v: %.3f)", a, v);
    }
}
