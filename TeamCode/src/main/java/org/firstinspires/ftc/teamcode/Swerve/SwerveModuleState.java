package org.firstinspires.ftc.teamcode.Swerve;

/**
 * data class for swerve modules target state
 * contains angle (degrees) and speed (-1 to 1).
 */
public class SwerveModuleState {
    public double angle;
    public double speed;

    public SwerveModuleState(double angle, double speed) {
        this.angle = angle;
        this.speed = speed;
    }

    public SwerveModuleState() {
        this(0, 0);
    }

    public SwerveModuleState optimize(double currentAngle) {
        double error = angleWrap(angle - currentAngle);

        if (Math.abs(error) > 90.0) {
            return new SwerveModuleState(angleWrap(angle + 180.0), -speed);
        }
        return new SwerveModuleState(angle, speed);
    }

    private static double angleWrap(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}
