package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * pedro pathing configuration constants.
 */
@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants();
    public static PinpointConstants pinpointConstants = new PinpointConstants();
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    static {
        // initialization of follower constants for swerve
        followerConstants.coefficientsDrivePIDF.setCoefficients(5, 0, 0.01, 0, 0);
        followerConstants.coefficientsTranslationalPIDF.setCoefficients(0.2, 0, 0.01, 0);
        followerConstants.coefficientsHeadingPIDF.setCoefficients(2, 0, 0.05, 0);
        
        followerConstants.forwardZeroPowerAcceleration = 1.5;
    }

    /**
     * create follower with custom swerve drivetrain and pinpoint localizer
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        SwerveDrivetrain drivetrain = new SwerveDrivetrain(hardwareMap);
        return new FollowerBuilder(followerConstants, hardwareMap)
                .setDrivetrain(drivetrain)
                .pinpointLocalizer(pinpointConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
