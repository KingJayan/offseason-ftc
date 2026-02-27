package org.firstinspires.ftc.teamcode.Swerve;

import org.firstinspires.ftc.teamcode.config.Constants;

/**swerve kinematics for 3-wheeled layout*/
public class Kinematics {
    private final double[] mX;
    private final double[] mY;

    public Kinematics() {
        double r = Constants.ROBOT_RADIUS;
        // left: 60 deg, right: -60 deg, back: 180 deg
        // x: right, y: forward
        mX = new double[] {
            r * Math.sin(Math.toRadians(60)),  // left
            r * Math.sin(Math.toRadians(-60)), // right
            r * Math.sin(Math.toRadians(180))  // back
        };
        mY = new double[] {
            r * Math.cos(Math.toRadians(60)),  // left
            r * Math.cos(Math.toRadians(-60)), // right
            r * Math.cos(Math.toRadians(180))  // back
        };
    }

    /**calc states from velocities*/
    public ModuleState[] calculate(double x, double y, double rx) {
        double driveMag = Math.hypot(x, y);
        double rotMag = Math.abs(rx);
        
        if (driveMag + rotMag > 1.0) {
            double scale = (1.0 - rotMag) / driveMag;
            x *= scale;
            y *= scale;
        }

        ModuleState[] states = new ModuleState[3];
        double max = 0.0;
        double[] wX = new double[3];
        double[] wY = new double[3];
        double[] wS = new double[3];
        double[] wA = new double[3];

        for (int i = 0; i < 3; i++) {
            double rX = -mY[i] * rx;
            double rY = mX[i] * rx;
            wX[i] = x + rX;
            wY[i] = y + rY;
            wS[i] = Math.hypot(wX[i], wY[i]);
            wA[i] = Math.toDegrees(Math.atan2(-wX[i], wY[i]));
            max = Math.max(max, wS[i]);
        }

        if (max > 1.0) {
            for (int i = 0; i < 3; i++) wS[i] /= max;
        }

        for (int i = 0; i < 3; i++) {
            double speed = wS[i] < Constants.MODULE_DB ? 0.0 : wS[i];
            states[i] = new ModuleState(wA[i], speed);
        }
        return states;
    }

    /**fcd transform*/
    public static double[] fcd(double x, double y, double heading) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        return new double[] {x * cos + y * sin, -x * sin + y * cos};
    }
}
