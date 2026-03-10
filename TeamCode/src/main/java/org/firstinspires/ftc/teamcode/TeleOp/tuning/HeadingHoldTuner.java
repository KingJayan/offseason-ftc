package org.firstinspires.ftc.teamcode.TeleOp.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Swerve.Drivetrain;
import org.firstinspires.ftc.teamcode.config.Config;
import org.firstinspires.ftc.teamcode.helpers.util.HeadingController;
import org.firstinspires.ftc.teamcode.helpers.util.MathUtil;

@TeleOp(name = "Heading Hold Tuner", group = "Tuning")
public class HeadingHoldTuner extends OpMode {
    private Drivetrain dt;
    private double tgtH = 0;
    private final HeadingController headingController = new HeadingController();
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        dt = new Drivetrain(hardwareMap);
        dt.resetYaw();
        headingController.setTarget(0, 0);
        timer.reset();
    }

    @Override
    public void loop() {
        dt.update();

        double curH = dt.getHeading();
        if (gamepad1.dpad_up) {
            tgtH = 0;
            headingController.setTarget(tgtH, curH);
            timer.reset();
        }
        if (gamepad1.dpad_right) {
            tgtH = -90;
            headingController.setTarget(tgtH, curH);
            timer.reset();
        }
        if (gamepad1.dpad_down) {
            tgtH = 180;
            headingController.setTarget(tgtH, curH);
            timer.reset();
        }
        if (gamepad1.dpad_left) {
            tgtH = 90;
            headingController.setTarget(tgtH, curH);
            timer.reset();
        }

        double err = MathUtil.wrap(tgtH - curH);
        double dtSec = timer.seconds();
        timer.reset();

        double rx = headingController.calculate(
                curH,
                dtSec,
                Config.H_KP,
                Config.H_KI,
                Config.H_KD,
                Config.alpha,
                Config.H_I_ZONE_DEG,
                Config.H_I_MAX,
                Config.H_I_LEAK_PER_SEC,
                Config.H_OUT_MAX
        );

        dt.drive(0, 0, rx, false);

        telemetry.addLine("=== Heading Hold Tuner ===");
        telemetry.addLine("press dpad to snap heading");
        telemetry.addLine();
        telemetry.addData("target", "%.1f", tgtH);
        telemetry.addData("current", "%.1f", curH);
        telemetry.addData("error", "%.1f", err);
        telemetry.addLine();
        telemetry.addData("H_KP", Config.H_KP);
        telemetry.addData("H_KI", Config.H_KI);
        telemetry.addData("integral", headingController.getIntegral());
        telemetry.addLine();
        telemetry.addLine("if wobbling: lower H_KP");
        telemetry.addLine("if oscillating: raise H_KD");
        telemetry.addLine("if steady error: raise H_KI a little");
        telemetry.update();
    }

    @Override
    public void stop() {
        dt.stop();
    }
}
