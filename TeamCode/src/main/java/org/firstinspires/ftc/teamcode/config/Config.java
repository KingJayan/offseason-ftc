package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Config {

    public static Mode T_MODE = Mode.SIN;
    public static Mode R_MODE = Mode.QUINT;
    
    public enum Mode { LINEAR, EXP, CUBIC_BLEND, QUINT, ROOT, CBRT, CIRC, SMOOTH, TANH, SIN, SIGMOID, LOG, BEZIER, SNIPER }

    //mode-based config
    public static double EXP_G = 2.0;
    public static double TANH_S = 2.0;
    public static double CUBIC_K = 0.8;
    private static double SIGMOID_K = 8.5;
    private static double LOG_K = 10;
    private static double BEZIER_P1 = 0.25;
    private static double LOW_POWER = 0.3;


    //slew-rate limiter vlaues
    public static boolean SLEW = true;
    public static double D_ACCEL = 5.5;
    public static double D_DECEL = 8.0;
    public static double R_ACCEL = 5.5;
    public static double R_DECEL = 10.0;

    //driver util
    public static boolean USE_MAG_SCALING = false;
    public static boolean USE_HEADING_HOLD = true;
    public static double PRECISION_SCALE = 0.4;
    public static double H_KP = 0.015;
    public static double H_KD = 0.002;
    
    //passive alignment
    public static boolean USE_PASSIVE_ALIGN = true;//global enable
    public static double PASSIVE_ALIGN_DEG = 2.0;//snap threshold
    
    //stall protection
    public static boolean USE_STALL_PROT = true;
    public static double STALL_TIMEOUT = 0.7;//seconds (increased for axon speed)
    public static double STALL_THRESHOLD = 2.0;//degrees

    /** apply deadband and curve to single value */
    public static double apply(double i, Mode m) {
        double abs = Math.abs(i);
        if (abs < Constants.STICK_DB) return 0;
        double r = (abs - Constants.STICK_DB) / (1 - Constants.STICK_DB);
        double o = 0;
        switch(m) {
            case LINEAR: o = r; break;
            case EXP: o = Math.pow(r, EXP_G); break;
            case CUBIC_BLEND:
                o = CUBIC_K * Math.pow(r, 3) + (1 - CUBIC_K) * r;
                break;
            case QUINT: //5th order quintic perlin
                o = r * r * r * (r * (r * 6 - 15) + 10);
                break;
            case ROOT: o = Math.sqrt(r); break;
            case CBRT: o = Math.cbrt(r); break;
            case CIRC: o = 1 - Math.sqrt(1 - r * r); break;
            case SMOOTH: o = r * r * (3 - 2 * r); break;
            case TANH: o = Math.tanh(r * TANH_S) / Math.tanh(TANH_S); break;
            case SIN: o = r - (Math.sin(2 * Math.PI * r) / (2 * Math.PI)); break;
            case SIGMOID: // norm log curve
                double rawSig = 1.0 / (1.0 + Math.exp(-SIGMOID_K * (r - 0.5)));
                double low = 1.0 / (1.0 + Math.exp(SIGMOID_K * 0.5));
                double high = 1.0 / (1.0 + Math.exp(-SIGMOID_K * 0.5));
                o = (rawSig - low) / (high - low);
                break;
            case LOG: // True Logarithmic Curve
                o = Math.log1p(LOG_K * r) / Math.log1p(LOG_K);
                break;
            case BEZIER: //quadd bezier
                o = 2 * (1 - r) * r * BEZIER_P1 + Math.pow(r, 2);
                break;
            case SNIPER: //piecewise sniper mode
                o = (r < 0.5) ? (r * LOW_POWER) : (LOW_POWER/2.0 + (r - 0.5) * (2 - LOW_POWER));
                break;
        }
        return Math.copySign(o, i);
    }

}
