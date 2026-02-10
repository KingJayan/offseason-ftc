package org.firstinspires.ftc.teamcode.Swerve;

/**
 * swerve drive kinematics calculations
 * converts robot velocity commands into individual module states
 * shared between teleop SwerveDrive and Pedro Pathing SwerveDrivetrain
 */
public class SwerveKinematics {
    //module poses (lf, rf, lb, rb)
    // positive x = forward, positive y = left
    private final double[] moduleX;
    private final double[] moduleY;

    public SwerveKinematics(double trackWidth, double wheelBase) {
        moduleX = new double[] {
            wheelBase / 2,// lf
            wheelBase / 2,// rf
            -wheelBase / 2,// lb
            -wheelBase / 2 // rb
        };
        moduleY = new double[] {
            trackWidth / 2,// lf
            -trackWidth / 2, // rf
            trackWidth / 2,// lb
            -trackWidth / 2// rb
        };
    }

    /**
     default constructor using SwerveConstants
     */
    public SwerveKinematics() {
        this(SwerveConstants.TRACK_WIDTH, SwerveConstants.WHEEL_BASE);
    }

    /**
     * calc module states from robot-centric velocities.
     * @param x strafe velocity (-1 to 1), positive = right
     * @param y forward velocity (-1 to 1), positive = forward
     * @param rx rotation velocity (-1 to 1), positive = counter-clockwise
     * @return array of 4 SwerveModuleState [lf, rf, lb, rb]
     */
    public SwerveModuleState[] calculate(double x, double y, double rx) {
        SwerveModuleState[] states = new SwerveModuleState[4];
        double maxSpeed = 0.0;

        double[] wheelX = new double[4];
        double[] wheelY = new double[4];
        double[] wheelSpeed = new double[4];
        double[] wheelAngle = new double[4];

        for (int i = 0; i < 4; i++) {
            // rotation contribution perp to module position
            // for ccw rot: vx = -ry * omega, vy = rx * omega
            double rotX = -moduleX[i] * rx;
            double rotY = moduleY[i] * rx;

            wheelX[i] = x + rotX;
            wheelY[i] = y + rotY;

            // mag + dir
            wheelSpeed[i] = Math.hypot(wheelX[i], wheelY[i]);
            wheelAngle[i] = Math.toDegrees(Math.atan2(wheelX[i], wheelY[i]));

            maxSpeed = Math.max(maxSpeed, wheelSpeed[i]);
        }

        if (maxSpeed > 1.0) {
            for (int i = 0; i < 4; i++) {
                wheelSpeed[i] /= maxSpeed;
            }
        }

        for (int i = 0; i < 4; i++) {
            states[i] = new SwerveModuleState(wheelAngle[i], wheelSpeed[i]);
        }

        return states;
    }

    /**
     * fcd to robo centric inputs
     */
    public static double[] fieldToRobot(double x, double y, double headingRad) {
        double cos = Math.cos(-headingRad);
        double sin = Math.sin(-headingRad);

        return new double[] {
            x * cos - y * sin,  // robot-relative strafe
            x * sin + y * cos   // robot-relative forward
        };
    }
}
