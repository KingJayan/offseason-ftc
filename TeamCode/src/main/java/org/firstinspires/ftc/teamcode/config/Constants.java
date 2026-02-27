package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**swerve hardware config*/
@Configurable
public class Constants {
    //ds names
    public static String L_DRIVE = "leftDrive";
    public static String R_DRIVE = "rightDrive";
    public static String B_DRIVE = "backDrive";

    public static String L_STEER = "leftSteer";
    public static String R_STEER = "rightSteer";
    public static String B_STEER = "backSteer";

    public static String L_ENC = "leftEnc";
    public static String R_ENC = "rightEnc";
    public static String B_ENC = "backEnc";

    public static String IMU = "imu";

    //directions
    public static boolean L_DRIVE_REV = false;
    public static boolean R_DRIVE_REV = false;
    public static boolean B_DRIVE_REV = false;

    public static boolean L_STEER_REV = false;
    public static boolean R_STEER_REV = false;
    public static boolean B_STEER_REV = false;

    //offsets
    public static double L_OFF = 0.0;
    public static double R_OFF = 0.0;
    public static double B_OFF = 0.0;

    //imu
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    //dims
    public static double ROBOT_RADIUS = 0.2; //dist from center to wheel (meters)

    //steer pid
    public static double KP = 0.02;
    public static double KD = 0.001;
    public static double KSTATIC = 0.05;
    public static double TOLERANCE = 2.0;

    //voltage
    public static double NOMINAL_VOLTAGE = 13.1;
    public static int VOLT_FILTER_N = 10;

    //deadbands
    public static double STICK_DB = 0.05;
    public static double MODULE_DB = 0.01;
    public static double STEER_JITTER_DEG = 0.5;

    //hardware
    public static double GEAR_RATIO = 3.2;
    public static double MAX_V = 3.3;
}
