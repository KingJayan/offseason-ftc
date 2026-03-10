package org.firstinspires.ftc.teamcode.helpers.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class HeadingControllerTest {

    @Test
    public void outputClampsToMax() {
        HeadingController c = new HeadingController();
        c.setTarget(180.0, 0.0);

        double out = c.calculate(0.0, 0.02, 0.1, 0.0, 0.0, 0.2, 30.0, 1.0, 0.0, 0.4);

        assertEquals(0.4, out, 1e-9);
    }

    @Test
    public void integralDoesNotGrowWhenSaturatedAndErrorPushesSameDirection() {
        HeadingController c = new HeadingController();
        c.setTarget(30.0, 0.0);

        for (int i = 0; i < 50; i++) {
            c.calculate(0.0, 0.02, 0.03, 0.8, 0.0, 0.2, 90.0, 2.0, 0.0, 0.25);
        }

        assertEquals(0.0, c.getIntegral(), 1e-6);
    }

    @Test
    public void integralResetsOnErrorSignChange() {
        HeadingController c = new HeadingController();
        c.setTarget(10.0, 0.0);

        for (int i = 0; i < 20; i++) {
            c.calculate(0.0, 0.02, 0.01, 0.4, 0.0, 0.2, 90.0, 2.0, 0.0, 1.0);
        }
        assertTrue(c.getIntegral() > 0.0);

        c.calculate(12.0, 0.02, 0.01, 0.4, 0.0, 0.2, 90.0, 2.0, 0.0, 1.0);

        assertEquals(0.0, c.getIntegral(), 1e-6);
    }
}

