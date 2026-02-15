package org.firstinspires.ftc.teamcode.Swerve;

public class MathUtils {
    public static double angleWrap(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}

