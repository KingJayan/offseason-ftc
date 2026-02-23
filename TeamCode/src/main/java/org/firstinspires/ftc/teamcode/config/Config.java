package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Config {
    public static Mode T_MODE = Mode.SIN;
    public static Mode R_MODE = Mode.QUINT;
    
    public enum Mode { LINEAR, EXP, QUAD, CUBIC, QUINT, ROOT, CBRT, CIRC, SMOOTH, TANH, SIN }

    //mode-based config
    public static double EXP_G = 2.0;
    public static double TANH_S = 2.0;
    public static double CUBIC_K = 0.8;

    //slew-rate limiter vlaues
    public static boolean SLEW = true;
    public static double D_ACCEL = 5.5;
    public static double D_DECEL = 8.0;
    public static double R_ACCEL = 5.5;
    public static double R_DECEL = 10.0;

    //driver util
    public static boolean USE_MAG_SCALING = false;
    public static double PRECISION_SCALE = 0.4;
    public static double H_KP = 0.015;
    public static double H_KD = 0.002;

    /**apply deadband and curve to single value*/
    public static double apply(double i, Mode m) {
        double abs = Math.abs(i);
        if (abs < Constants.STICK_DB) return 0;
        double r = (abs - Constants.STICK_DB) / (1 - Constants.STICK_DB);
        double o = 0;
        switch(m) {
            case LINEAR: o = r; break;
            case EXP: o = Math.pow(r, EXP_G); break;
            case QUAD: o = r * r; break;
            case CUBIC: 
                o = CUBIC_K * Math.pow(r, 3) + (1 - CUBIC_K) * r;
                break;
            case QUINT:
                o = r * r * r * (r * (r * 6 - 15) + 10); 
                break;
            case ROOT: o = Math.sqrt(r); break;
            case CBRT: o = Math.cbrt(r); break;
            case CIRC: o = 1 - Math.sqrt(1 - r * r); break;
            case SMOOTH: o = r * r * (3 - 2 * r); break;
            case TANH: o = Math.tanh(r * TANH_S) / Math.tanh(TANH_S); break;
            case SIN: o = r - (Math.sin(2 * Math.PI * r) / (2 * Math.PI)); break;
        }
        return Math.copySign(o, i);
    }
}
