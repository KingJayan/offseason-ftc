package org.firstinspires.ftc.teamcode.helpers.util;

import com.qualcomm.robotcore.util.ElapsedTime;

/**asymmetric slew rate limiter*/
public class RateLimiter {
    private double accel, decel, last = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public RateLimiter(double accel, double decel) {
        this.accel = accel;
        this.decel = decel;
    }

    public double calculate(double target) {
        double dt = timer.seconds();
        timer.reset();
        if (dt > 0.2) return target;

        double rate = Math.abs(target) > Math.abs(last) ? accel : decel;
        double step = rate * dt;
        last += Math.max(-step, Math.min(step, target - last));
        return last;
    }

    public void setRates(double accel, double decel) {
        this.accel = accel;
        this.decel = decel;
    }

    public void reset(double value) {
        last = value;
        timer.reset();
    }
}
