package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Config;
import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.Swerve.Drivetrain;
import org.firstinspires.ftc.teamcode.helpers.util.RateLimiter;
import org.firstinspires.ftc.teamcode.helpers.util.HeadingController;
import org.firstinspires.ftc.teamcode.helpers.util.MathUtil;
import org.firstinspires.ftc.teamcode.helpers.Toggle;

@TeleOp(name="swerve")
public class SwerveOp extends OpMode {
    private Drivetrain dt;
    private enum DriveMode { ROBOT, FIELD }
    private DriveMode mode = DriveMode.FIELD;
    
    private final Toggle modeToggle = new Toggle();
    private final Toggle snapUpToggle = new Toggle();
    private final Toggle snapLeftToggle = new Toggle();
    private final Toggle snapDownToggle = new Toggle();
    private final Toggle snapRightToggle = new Toggle();
    private final Toggle yawResetToggle = new Toggle();
    private final RateLimiter rX = new RateLimiter(Config.D_ACCEL, Config.D_DECEL);
    private final RateLimiter rY = new RateLimiter(Config.D_ACCEL, Config.D_DECEL);
    private final RateLimiter rR = new RateLimiter(Config.R_ACCEL, Config.R_DECEL);

    private final HeadingController headingController = new HeadingController();
    private boolean hLock = false;
    private boolean lastRXNeutral = true;
    private boolean wasDefense = false;
    private final ElapsedTime hTimer = new ElapsedTime();
    private double lastModeToggleAt = -10.0;
    private double lastSnapAt = -10.0;
    private double lastYawResetAt = -10.0;

    private double tgtH = 0;

    @Override
    public void init() {
        dt = new Drivetrain(hardwareMap);
        dt.resetYaw();
        tgtH = dt.getHeading();
        headingController.setTarget(tgtH, tgtH);
        hTimer.reset();
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
        double rRaw = gamepad1.right_stick_x;

        if (modeToggle.momentary(gamepad1.ps) && (getRuntime() - lastModeToggleAt) >= Config.MODE_TOGGLE_DB_SEC) {
            lastModeToggleAt = getRuntime();
            mode = (mode == DriveMode.ROBOT) ? DriveMode.FIELD : DriveMode.ROBOT;
            gamepad1.rumble(150);
        }

        if (gamepad1.b) {
            dt.defense();
            hLock = false;
            wasDefense = true;
            return;
        } else if (wasDefense) {
            dt.drive(0, 0, 0, mode == DriveMode.FIELD);
            wasDefense = false;
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

        double rx = Config.apply(rRaw, Config.R_MODE);

        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            x *= Config.PRECISION_SCALE;
            y *= Config.PRECISION_SCALE;
            rx *= Config.PRECISION_SCALE;
        }

        double curH = dt.getHeading();
        boolean stickMoving = Math.abs(rRaw) > Constants.STICK_DB;

        if (snapUpToggle.momentary(gamepad1.dpad_up) && (getRuntime() - lastSnapAt) >= Config.SNAP_DB_SEC) {
            lastSnapAt = getRuntime();
            tgtH = 0;
            hLock = true;
            headingController.setTarget(tgtH, curH);
            hTimer.reset();
            gamepad1.rumble(100);
        }
        else if (snapLeftToggle.momentary(gamepad1.dpad_left) && (getRuntime() - lastSnapAt) >= Config.SNAP_DB_SEC) {
            lastSnapAt = getRuntime();
            tgtH = 90;
            hLock = true;
            headingController.setTarget(tgtH, curH);
            hTimer.reset();
            gamepad1.rumble(100);
        }
        else if (snapDownToggle.momentary(gamepad1.dpad_down) && (getRuntime() - lastSnapAt) >= Config.SNAP_DB_SEC) {
            lastSnapAt = getRuntime();
            tgtH = 180;
            hLock = true;
            headingController.setTarget(tgtH, curH);
            hTimer.reset();
            gamepad1.rumble(100);
        }
        else if (snapRightToggle.momentary(gamepad1.dpad_right) && (getRuntime() - lastSnapAt) >= Config.SNAP_DB_SEC) {
            lastSnapAt = getRuntime();
            tgtH = -90;
            hLock = true;
            headingController.setTarget(tgtH, curH);
            hTimer.reset();
            gamepad1.rumble(100);
        }

        if (Config.USE_PASSIVE_ALIGN && !stickMoving && !hLock) {
            double[] cards = {0, 90, 180, -90};
            for (double c : cards) {
                if (Math.abs(MathUtil.wrap(c - curH)) < Config.PASSIVE_ALIGN_DEG) {
                    tgtH = c;
                    hLock = true;
                    headingController.setTarget(tgtH, curH);
                    hTimer.reset();
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
                headingController.setTarget(tgtH, curH);
                hTimer.reset();
                lastRXNeutral = true;
            }
        } else if (stickMoving) {
            hLock = false;
        }

        if (hLock) {
            double dtH = hTimer.seconds();
            hTimer.reset();
            rx = headingController.calculate(
                    curH,
                    dtH,
                    Config.H_KP,
                    Config.H_KI,
                    Config.H_KD,
                    Config.alpha,
                    Config.H_I_ZONE_DEG,
                    Config.H_I_MAX,
                    Config.H_I_LEAK_PER_SEC,
                    Config.H_OUT_MAX
            );
        } else {
            hTimer.reset();
        }

        if (yawResetToggle.momentary(gamepad1.x) && (getRuntime() - lastYawResetAt) >= Config.YAW_RESET_DB_SEC) {
            lastYawResetAt = getRuntime();
            dt.resetYaw();
            tgtH = dt.getHeading();
            headingController.setTarget(tgtH, tgtH);
            hTimer.reset();
            gamepad1.rumble(200);
        }

        if (Config.SLEW) {
            x = rX.calculate(x);
            y = rY.calculate(y);
            rx = rR.calculate(rx);
        }

        dt.drive(x, y, rx, mode == DriveMode.FIELD);

        telemetry.addData("mode", mode);
        telemetry.addData("h", "%.1f", curH);
        telemetry.addData("l", "%.1f", dt.getL().getCurDeg());
        telemetry.addData("r", "%.1f", dt.getR().getCurDeg());
        telemetry.addData("b", "%.1f", dt.getB().getCurDeg());
        telemetry.update();
    }

    @Override
    public void stop() { dt.stop(); }
}
