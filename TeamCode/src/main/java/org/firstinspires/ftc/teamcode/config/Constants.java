package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**swerve hardware config*/
@Configurable
public class Constants {
    //ds names
    public static String LF_DRIVE = "leftFront";
    public static String RF_DRIVE = "rightFront";
    public static String LB_DRIVE = "leftBack";
    public static String RB_DRIVE = "rightBack";

    public static String LF_STEER = "steerFL";
    public static String RF_STEER = "steerFR";
    public static String LB_STEER = "steerBL";
    public static String RB_STEER = "steerBR";

    public static String LF_ENC = "sensorFL";
    public static String RF_ENC = "sensorFR";
    public static String LB_ENC = "sensorBL";
    public static String RB_ENC = "sensorBR";

    public static String IMU = "imu";

    //directions
    public static boolean LF_DRIVE_REV = false;
    public static boolean RF_DRIVE_REV = false;
    public static boolean LB_DRIVE_REV = false;
    public static boolean RB_DRIVE_REV = false;

    public static boolean LF_STEER_REV = false;
    public static boolean RF_STEER_REV = false;
    public static boolean LB_STEER_REV = false;
    public static boolean RB_STEER_REV = false;

    //offsets
    public static double LF_OFF = 0.0;
    public static double RF_OFF = 0.0;
    public static double LB_OFF = 0.0;
    public static double RB_OFF = 0.0;

    //imu
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    //dims
    public static double TRACK_WIDTH = 0.35;
    public static double WHEEL_BASE = 0.35;

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
