package org.firstinspires.ftc.teamcode.helpers.util;

public class HeadingController {
    private double targetDeg = 0.0;
    private double lastErr = 0.0;
    private double lastD = 0.0;
    private double integral = 0.0;
    private boolean initialized = false;

    public void setTarget(double targetDeg, double currentHeadingDeg) {
        this.targetDeg = targetDeg;
        this.lastErr = MathUtil.wrap(targetDeg - currentHeadingDeg);
        this.lastD = 0.0;
        this.integral = 0.0;
        this.initialized = true;
    }

    public double getTargetDeg() {
        return targetDeg;
    }

    public double getIntegral() {
        return integral;
    }

    public double calculate(double currentHeadingDeg, double dtSec, double kP, double kI, double kD, double alpha, double iZoneDeg, double iMax, double iLeakPerSec, double outMax) {
        if (!initialized) {
            setTarget(currentHeadingDeg, currentHeadingDeg);
        }

        double err = MathUtil.wrap(targetDeg - currentHeadingDeg);
        double safeDt = Math.max(0.0, dtSec);
        double clampedAlpha = clamp(alpha, 0.0, 1.0);

        double rawD = safeDt > 0.0 ? (err - lastErr) / safeDt : 0.0;
        double d = (clampedAlpha * rawD) + ((1.0 - clampedAlpha) * lastD);

        if (Math.signum(err) != Math.signum(lastErr) && Math.abs(lastErr) > 1e-6) {
            integral = 0.0;
        }

        double leakScale = Math.max(0.0, 1.0 - (Math.max(0.0, iLeakPerSec) * safeDt));

        if (safeDt > 0.0) {
            if (Math.abs(err) <= Math.max(0.0, iZoneDeg)) {
                integral *= leakScale;
                double candidateIntegral = clamp(integral + (err * safeDt), -Math.abs(iMax), Math.abs(iMax));
                double unsat = (err * kP) + (candidateIntegral * kI) + (d * kD);
                boolean saturated = Math.abs(unsat) > Math.abs(outMax);
                boolean pushesFurtherIntoSat = (Math.signum(err) != 0.0) && (Math.signum(err) == Math.signum(unsat));
                if (!saturated || !pushesFurtherIntoSat) {
                    integral = candidateIntegral;
                }
            } else {
                integral *= leakScale;
            }
        }

        double out = (err * kP) + (integral * kI) + (d * kD);
        out = clamp(out, -Math.abs(outMax), Math.abs(outMax));

        lastErr = err;
        lastD = d;
        return out;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}

