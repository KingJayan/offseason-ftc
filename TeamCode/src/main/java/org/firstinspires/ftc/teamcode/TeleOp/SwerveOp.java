package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Config;
import org.firstinspires.ftc.teamcode.Swerve.Drivetrain;
import org.firstinspires.ftc.teamcode.helpers.util.RateLimiter;
import org.firstinspires.ftc.teamcode.helpers.util.MathUtil;
import org.firstinspires.ftc.teamcode.helpers.Toggle;

@TeleOp(name="swerve")
public class SwerveOp extends OpMode {
    private Drivetrain dt;
    private enum DriveMode { ROBOT, FIELD }
    private DriveMode mode = DriveMode.FIELD;
    
    private final Toggle modeToggle = new Toggle();
    private final RateLimiter rX = new RateLimiter(Config.D_ACCEL, Config.D_DECEL);
    private final RateLimiter rY = new RateLimiter(Config.D_ACCEL, Config.D_DECEL);
    private final RateLimiter rR = new RateLimiter(Config.R_ACCEL, Config.R_DECEL);

    private double tgtH = 0;
    private boolean hLock = false;
    private double lastHErr = 0;
    private boolean lastRXNeutral = true;
    private final ElapsedTime hTimer = new ElapsedTime();

    @Override
    public void init() {
        dt = new Drivetrain(hardwareMap);
        dt.resetYaw();
        tgtH = dt.getHeading();
    }

    @Override
    public void init_loop() { dt.update(); }

    @Override
    public void loop() {
        dt.update();
        rX.setRates(Config.D_ACCEL, Config.D_DECEL);
        rY.setRates(Config.D_ACCEL, Config.D_DECEL);
        rR.setRates(Config.R_ACCEL, Config.R_DECEL);

        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        if (modeToggle.update(gamepad1.ps)) {
            mode = (mode == DriveMode.ROBOT) ? DriveMode.FIELD : DriveMode.ROBOT;
            gamepad1.rumble(150);
        }

        if (gamepad1.b) {
            dt.defense();
            hLock = false;
            return;
        }

        double x, y;
        if (Config.USE_MAG_SCALING) {
            double mag = Math.hypot(lx, ly);
            if (mag > 1.0) { lx /= mag; ly /= mag; mag = 1.0; }
            double scale = mag > 0 ? Config.apply(mag, Config.T_MODE) / mag : 0;
            x = lx * scale; y = ly * scale;
        } else {
            x = Config.apply(lx, Config.T_MODE);
            y = Config.apply(ly, Config.T_MODE);
        }

        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            x *= Config.PRECISION_SCALE;
            y *= Config.PRECISION_SCALE;
            rx *= Config.PRECISION_SCALE;
        }

        double curH = dt.getHeading();
        boolean stickMoving = Math.abs(rx) > 0.05;

        if (gamepad1.dpad_up) { tgtH = 0; hLock = true; gamepad1.rumble(100); }
        else if (gamepad1.dpad_left) { tgtH = 90; hLock = true; gamepad1.rumble(100); }
        else if (gamepad1.dpad_down) { tgtH = 180; hLock = true; gamepad1.rumble(100); }
        else if (gamepad1.dpad_right) { tgtH = -90; hLock = true; gamepad1.rumble(100); }

        //passive alignment
        if (Config.USE_PASSIVE_ALIGN && !stickMoving && !hLock) {
            double absH = Math.abs(curH % 360);
            double[] cardinals = {0, 90, 180, 270, -90, -180, -270};
            for (double c : cardinals) {
                if (Math.abs(MathUtil.wrap(c - curH)) < Config.PASSIVE_ALIGN_DEG) {
                    tgtH = c;
                    hLock = true;
                    gamepad1.rumble(100);
                    break;
                }
            }
        }

        if (Config.USE_HEADING_HOLD) {
            if (stickMoving) {
                hLock = false;
                lastRXNeutral = false;
            } else if (!lastRXNeutral) {
                tgtH = curH;
                hLock = true;
                lastRXNeutral = true;
            }
        } else if (stickMoving) {
            hLock = false;
        }

        if (hLock) {
            double dtH = hTimer.seconds(); hTimer.reset();
            double err = MathUtil.wrap(tgtH - curH);
            double d = dtH > 0 ? (err - lastHErr) / dtH : 0;
            lastHErr = err;
            rx = (err * Config.H_KP) + (d * Config.H_KD);
        }

        if (gamepad1.x) {
            dt.resetYaw();
            gamepad1.rumble(200);
        }

        if (Config.SLEW) {
            x = rX.calculate(x);
            y = rY.calculate(y);
            rx = rR.calculate(rx);
        }

        dt.drive(x, y, rx, mode != DriveMode.ROBOT);

        telemetry.addData("mode", mode);
        telemetry.addData("h", "%.1f", curH);
        telemetry.addData("lock", hLock ? "active (" + (int)tgtH + ")" : "off");
        telemetry.update();
    }

    @Override
    public void stop() { dt.stop(); }
}
