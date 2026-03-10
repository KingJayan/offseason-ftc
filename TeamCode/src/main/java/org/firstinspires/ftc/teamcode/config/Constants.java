package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Configurable
public class Constants {
    //drive motors
    public static String L_DRIVE = "lDrive";
    public static String R_DRIVE = "rDrive";
    public static String B_DRIVE = "bDrive";

    //steering servos
    public static String L_STEER = "lSteer";
    public static String R_STEER = "rSteer";
    public static String B_STEER = "bSteer";

    //steering analog feedback
    public static String L_STEER_ENC = "lSteerEnc";
    public static String R_STEER_ENC = "rSteerEnc";
    public static String B_STEER_ENC = "bSteerEnc";

    public static String IMU = "imu";

    //motor directions (true = reversed)
    public static boolean L_DRIVE_REV = false;
    public static boolean R_DRIVE_REV = false;
    public static boolean B_DRIVE_REV = false;

    //servo directions
    public static boolean L_STEER_REV = false;
    public static boolean R_STEER_REV = false;
    public static boolean B_STEER_REV = false;

    //servo offsets
    public static double L_OFF_DEG = 0.0;
    public static double R_OFF_DEG = 0.0;
    public static double B_OFF_DEG = 0.0;

    //imu orientation
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    //dimensions
    public static double ROBOT_RADIUS_METERS = 0.2;
    public static double ROT_SCALER = 4.0;

    //system defaults
    public static double NOMINAL_VOLTAGE = 13.1;
    public static int VOLT_FILTER_N = 10;

    //deadbands
    public static double STICK_DB = 0.05;
    public static double MODULE_DB = 0.01;
    public static double STEER_JITTER_DEG = 0.5;
    public static double TOLERANCE_DEG = 2.0;

    //steer crservo control
    public static double STEER_ANALOG_MAX_V = 3.3;
    public static double STEER_KP = 0.012;
    public static double STEER_MAX_PWR = 0.85;

    //servo config
    public static double GEAR_RATIO = 3.2;
}
