package org.firstinspires.ftc.teamcode.TeleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Swerve.Drivetrain;

/**verifies imu orientation and heading logic*/
@TeleOp(name="swerve imu test")
public class SwerveIMUTest extends OpMode {
    private Drivetrain dt;

    @Override
    public void init() {
        dt = new Drivetrain(hardwareMap);
    }

    @Override
    public void loop() {
        double h = dt.getHeading();
        
        telemetry.addLine("--- imu test ---");
        telemetry.addLine("rotate the robot ccw (to the left)");
        telemetry.addLine("heading should increase (+)");
        telemetry.addLine();
        telemetry.addData("heading", "%.2f", h);
        
        if (gamepad1.a) {
            dt.resetYaw();
            telemetry.addLine("yaw reset!");
        }
        
        telemetry.update();
    }
}
