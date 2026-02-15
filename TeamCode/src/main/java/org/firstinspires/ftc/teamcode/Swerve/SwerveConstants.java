package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * swerve hardware configuration
 */
public class SwerveConstants {

    //DS configuration names

    public static String LEFT_FRONT_MOTOR = "leftFront";
    public static String RIGHT_FRONT_MOTOR = "rightFront";
    public static String LEFT_BACK_MOTOR = "leftBack";
    public static String RIGHT_BACK_MOTOR = "rightBack";

    public static String STEER_LF = "steerFL";
    public static String STEER_RF = "steerFR";
    public static String STEER_LB = "steerBL";
    public static String STEER_RB = "steerBR";

    public static String SENSOR_LF = "sensorFL";
    public static String SENSOR_RF = "sensorFR";
    public static String SENSOR_LB = "sensorBL";
    public static String SENSOR_RB = "sensorBR";

    public static String IMU_NAME = "imu";



    //motor and servo direction configu, true->reversed

    public static boolean DRIVE_LF_REVERSED = false;
    public static boolean DRIVE_RF_REVERSED = false;
    public static boolean DRIVE_LB_REVERSED = false;
    public static boolean DRIVE_RB_REVERSED = false;

    public static boolean STEER_LF_REVERSED = false;
    public static boolean STEER_RF_REVERSED = false;
    public static boolean STEER_LB_REVERSED = false;
    public static boolean STEER_RB_REVERSED = false;

    //IMU orientation on robo
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;


    // dimensions

    public static double TRACK_WIDTH = 0.35;// L-R distance (meters)
    public static double WHEEL_BASE = 0.35;// F-B distance (meters)


    //STEER pid tuning TODO: tune these
    public static double STEER_KP = 0.02;
    public static double STEER_KD = 0.001;
    public static double STEER_KSTATIC = 0.05;//static friction comp
    public static double STEER_TOLERANCE = 2.0;//degrees

    //voltage compensation
    public static double NOMINAL_VOLTAGE = 13.1;

    //all deadbands
    public static double INPUT_DEADBAND = 0.05; // joystick
    public static double SPEED_DEADBAND = 0.01; // module speed

    //module hardware/misc
    public static double GEAR_RATIO = 3.2;//servo:module ratio
    public static double MAX_SENSOR_VOLTAGE = 3.3; // analog encoder max
}
