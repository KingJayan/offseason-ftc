package org.firstinspires.ftc.teamcode.Swerve;

import org.firstinspires.ftc.teamcode.config.Constants;

/**swerve kinematics*/
public class Kinematics {
    private final double[] mX;
    private final double[] mY;

    public Kinematics() {
        mX = new double[] {Constants.WHEEL_BASE/2, Constants.WHEEL_BASE/2, -Constants.WHEEL_BASE/2, -Constants.WHEEL_BASE/2};
        mY = new double[] {Constants.TRACK_WIDTH/2, -Constants.TRACK_WIDTH/2, Constants.TRACK_WIDTH/2, -Constants.TRACK_WIDTH/2};
    }

    /**calc states from velocities*/
    public ModuleState[] calculate(double x, double y, double rx) {
        ModuleState[] states = new ModuleState[4];
        double max = 0.0;
        double[] wX = new double[4];
        double[] wY = new double[4];
        double[] wS = new double[4];
        double[] wA = new double[4];

        for (int i = 0; i < 4; i++) {
            double rX = -mY[i] * rx;
            double rY = mX[i] * rx;
            wX[i] = x + rX;
            wY[i] = y + rY;
            wS[i] = Math.hypot(wX[i], wY[i]);
            wA[i] = Math.toDegrees(Math.atan2(-wX[i], wY[i]));
            max = Math.max(max, wS[i]);
        }

        if (max > 1.0) {
            for (int i = 0; i < 4; i++) wS[i] /= max;
        }

        for (int i = 0; i < 4; i++) {
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
