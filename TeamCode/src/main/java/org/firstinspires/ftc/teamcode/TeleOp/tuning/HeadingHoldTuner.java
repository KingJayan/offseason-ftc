package org.firstinspires.ftc.teamcode.TeleOp.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Swerve.Drivetrain;
import org.firstinspires.ftc.teamcode.config.Config;
import org.firstinspires.ftc.teamcode.helpers.util.MathUtil;

@TeleOp(name = "Heading Hold Tuner", group = "Tuning")
public class HeadingHoldTuner extends OpMode {
    private Drivetrain dt;
    private double tgtH = 0;
    private double lastErr = 0;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        dt = new Drivetrain(hardwareMap);
        dt.resetYaw();
    }

    @Override
    public void loop() {
        dt.update();

        if (gamepad1.dpad_up) tgtH = 0;
        if (gamepad1.dpad_right) tgtH = -90;
        if (gamepad1.dpad_down) tgtH = 180;
        if (gamepad1.dpad_left) tgtH = 90;

        double curH = dt.getHeading();
        double err = MathUtil.wrap(tgtH - curH);
        double dtSec = timer.seconds();
        timer.reset();

        double d = dtSec > 0 ? (err - lastErr) / dtSec : 0;
        lastErr = err;

        double rx = (err * Config.H_KP) + (d * Config.H_KD);

        dt.drive(0, 0, rx, false);

        telemetry.addLine("=== Heading Hold Tuner ===");
        telemetry.addLine("press dpad to snap heading");
        telemetry.addLine();
        telemetry.addData("target", "%.1f", tgtH);
        telemetry.addData("current", "%.1f", curH);
        telemetry.addData("error", "%.1f", err);
        telemetry.addLine();
        telemetry.addData("H_KP", Config.H_KP);
        telemetry.addData("H_KD", Config.H_KD);
        telemetry.addLine();
        telemetry.addLine("if wobbling: lower H_KP");
        telemetry.addLine("if oscillating: raise H_KD");
        telemetry.update();
    }

    @Override
    public void stop() {
        dt.stop();
    }
}

