package org.firstinspires.ftc.teamcode.helpers.util;

import org.firstinspires.ftc.teamcode.config.Constants;
import java.util.LinkedList;
import java.util.Queue;

/**rolling volt filter and comp logic*/
public class VComp {
    private final Queue<Double> q = new LinkedList<>();
    private double sum = 0;

    /**update filter and get comp scalar*/
    public double get(double v) {
        sum += v;
        q.add(v);
        if (q.size() > Constants.VOLT_FILTER_N) {
            Double p = q.poll();
            if (p != null) sum -= p;
        }
        double avg = !q.isEmpty() ? sum / q.size() : Constants.NOMINAL_VOLTAGE;
        if (avg <= 0 || Double.isNaN(avg) || Double.isInfinite(avg)) return 1.0;
        return Constants.NOMINAL_VOLTAGE / avg;
    }
}
