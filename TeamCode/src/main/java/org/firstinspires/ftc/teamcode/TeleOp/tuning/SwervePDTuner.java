package org.firstinspires.ftc.teamcode.TeleOp.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Config;
import org.firstinspires.ftc.teamcode.Swerve.Drivetrain;
import org.firstinspires.ftc.teamcode.helpers.util.MathUtil;

/**
 * opmode to tune the heading pd controller.
 * uses Config.H_KP and Config.H_KD live via dashboard.
 */
@TeleOp(name="swerve pd tuner")
public class SwervePDTuner extends OpMode {
    private Drivetrain dt;
    private double tgtH = 0;
    private double lastErr = 0;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        dt = new Drivetrain(hardwareMap);
        dt.resetYaw();
        tgtH = 0;
    }

    @Override
    public void loop() {
        dt.update();

        // snap to cardinal directions for testing
        if (gamepad1.dpad_up) tgtH = 0;
        if (gamepad1.dpad_right) tgtH = -90;
        if (gamepad1.dpad_down) tgtH = 180;
        if (gamepad1.dpad_left) tgtH = 90;

        // pd logic (matching SwerveOp)
        double curH = dt.getHeading();
        double err = MathUtil.wrap(tgtH - curH);
        double dtSec = timer.seconds();
        timer.reset();

        double d = dtSec > 0 ? (err - lastErr) / dtSec : 0;
        lastErr = err;
        
        double rx = (err * Config.H_KP) + (d * Config.H_KD);

        // only rotation, no translation
        dt.drive(0, 0, rx, false);

        telemetry.addData("target", "%.1f", tgtH);
        telemetry.addData("current", "%.1f", curH);
        telemetry.addData("error", "%.1f", err);
        telemetry.addData("kP", Config.H_KP);
        telemetry.addData("kD", Config.H_KD);
        telemetry.addLine("\nuse dpad to change target");
        telemetry.update();
    }
}
