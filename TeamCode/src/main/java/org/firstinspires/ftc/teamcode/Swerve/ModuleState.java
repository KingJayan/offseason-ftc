package org.firstinspires.ftc.teamcode.Swerve;

import org.firstinspires.ftc.teamcode.helpers.util.MathUtil;

/**module target state*/
public class ModuleState {
    public double angle;
    public double speed;

    public ModuleState(double angle, double speed) {
        this.angle = angle;
        this.speed = speed;
    }

    public ModuleState() {
        this(0, 0);
    }

    public ModuleState optimize(double current) {
        double err = MathUtil.wrap(angle - current);
        if (Math.abs(err) > 90.0) {
            return new ModuleState(MathUtil.wrap(angle + 180.0), -speed);
        }
        return new ModuleState(angle, speed);
    }
}
