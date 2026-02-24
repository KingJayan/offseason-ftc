package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**basic autonomous path to verify pedropathing swerve drivetrain*/
@Autonomous(name="basic auto", group="auto")
public class BasicAuto extends LinearOpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        //define a simple s-curve path chain
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(0, 0, 0), new Pose(24, 0, 0)))
                .setLinearHeadingInterpolation(0, Math.toRadians(90))
                .addPath(new BezierLine(new Pose(24, 0, Math.toRadians(90)), new Pose(24, 24, Math.toRadians(90))))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        telemetry.addLine("initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("h", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}
