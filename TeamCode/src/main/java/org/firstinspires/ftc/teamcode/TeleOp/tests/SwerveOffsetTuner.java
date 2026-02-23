package org.firstinspires.ftc.teamcode.TeleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.config.Constants;

/**
 * opmode to find absolute encoder offsets.
 * 1. physically align all wheels so they are pointing perfectly forward.
 * 2. record the "raw" values shown on telemetry.
 * 3. enter these values into Constants.java (LF_OFF, RF_OFF, etc.).
 */
@TeleOp(name="swerve offset tuner")
public class SwerveOffsetTuner extends OpMode {
    private AnalogInput lf, rf, lb, rb;

    @Override
    public void init() {
        lf = hardwareMap.get(AnalogInput.class, Constants.LF_ENC);
        rf = hardwareMap.get(AnalogInput.class, Constants.RF_ENC);
        lb = hardwareMap.get(AnalogInput.class, Constants.LB_ENC);
        rb = hardwareMap.get(AnalogInput.class, Constants.RB_ENC);
    }

    @Override
    public void loop() {
        telemetry.addLine("--- swerve offset tuner ---");
        telemetry.addLine("align all wheels forward and record these values:");
        telemetry.addLine();
        
        showRaw("LF", lf);
        showRaw("RF", rf);
        showRaw("LB", lb);
        showRaw("RB", rb);
        
        telemetry.update();
    }

    private void showRaw(String name, AnalogInput sensor) {
        double voltage = sensor.getVoltage();
        double angle = (voltage / Constants.MAX_V) * 360.0;
        telemetry.addData(name, "raw angle: %.2f (v: %.3f)", angle, voltage);
    }
}
