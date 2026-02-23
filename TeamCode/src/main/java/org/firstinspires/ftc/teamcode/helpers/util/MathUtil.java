package org.firstinspires.ftc.teamcode.helpers.util;

public class MathUtil {
    public static double wrap(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}
