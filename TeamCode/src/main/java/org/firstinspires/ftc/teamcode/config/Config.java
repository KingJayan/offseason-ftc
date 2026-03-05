package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Config {

    /// TODO: TUNE
    // heading hold, snap2heading pid values
    public static double H_KP = 0.015;
    public static double H_KI = 0.0; //rais by tiny amts if there is steady state error
    public static double H_KD = 0.002;
    public static double alpha = 0.23; //lowpassfilter alpha -- more is less smoothing, less is more smoothing

    // slew rate values
    public static double D_ACCEL = 5.5;
    public static double D_DECEL = 8.0;
    public static double R_ACCEL = 5.5;
    public static double R_DECEL = 10.0;

    // stick curves
    public static Mode T_MODE = Mode.SIN;
    public static Mode R_MODE = Mode.QUINT;

    public enum Mode { LINEAR, EXP, QUINT, ROOT, CBRT, TANH, SIN }

    // curve parameters
    public static double EXP_G = 2.0;
    public static double TANH_S = 2.0;

    // drive behavior
    public static boolean SLEW = true;
    public static boolean USE_MAG_SCALING = false;
    public static boolean USE_HEADING_HOLD = true;
    public static double PRECISION_SCALE = 0.4;

    // passive align, usually off in scenarious where imu drift common
    public static boolean USE_PASSIVE_ALIGN = false;
    public static double PASSIVE_ALIGN_DEG = 2.0;

    // stall prot
    // disabled by default (no steering feedback in current servo path)
    public static boolean USE_STALL_PROT = false;
    public static double STALL_TIMEOUT = 0.7;
    public static double STALL_THRESHOLD = 2.0;

    public static double apply(double i, Mode m) {
        double abs = Math.abs(i);
        if (abs < Constants.STICK_DB) return 0;
        double r = (abs - Constants.STICK_DB) / (1 - Constants.STICK_DB);
        double o = 0;
        switch(m) {
            case LINEAR: o = r; break;
            case EXP: o = Math.pow(r, EXP_G); break;
            case QUINT: o = r * r * r * (r * (r * 6 - 15) + 10); break;
            case ROOT: o = Math.sqrt(r); break;
            case CBRT: o = Math.cbrt(r); break;
            case TANH: o = Math.tanh(r * TANH_S) / Math.tanh(TANH_S); break;
            case SIN: o = r - (Math.sin(2 * Math.PI * r) / (2 * Math.PI)); break;
        }
        return Math.copySign(o, i);
    }

}
