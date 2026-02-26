package org.firstinspires.ftc.teamcode.Swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class SwerveMathTest {

    @Test
    public void testKinematicsForward() {
        Kinematics kin = new Kinematics();
        //move straight forward
        ModuleState[] states = kin.calculate(0, 1, 0);
        
        for (ModuleState s : states) {
            assertEquals(0.0, s.angle, 0.01);
            assertEquals(1.0, s.speed, 0.01);
        }
    }

    @Test
    public void testKinematicsStrafe() {
        Kinematics kin = new Kinematics();
        //strafe right (positive x w/ kin.)
        ModuleState[] states = kin.calculate(1, 0, 0);
        
        for (ModuleState s : states) {
            assertEquals(-90.0, s.angle, 0.01);
            assertEquals(1.0, s.speed, 0.01);
        }
    }

    @Test
    public void testOptimizationShortestPath() {
        ModuleState target = new ModuleState(170, 1.0);
        //if current is -170, error is -20, no flip needed
        ModuleState opt1 = target.optimize(-170);
        assertEquals(170.0, opt1.angle, 0.01);
        assertEquals(1.0, opt1.speed, 0.01);

        //if current is 0, target 170 is > 90 away, should flip
        ModuleState opt2 = target.optimize(0);
        assertEquals(-10.0, opt2.angle, 0.01);
        assertEquals(-1.0, opt2.speed, 0.01);
    }

    @Test
    public void testKinematicsRotation() {
        Kinematics kin = new Kinematics();
        //pure ccw rotation
        ModuleState[] states = kin.calculate(0, 0, 1);
        
        //lf (top-left) should point back-left (-135 or 225)
        //rf (top-right) should point forward-left (45)
        //lb (bottom-left) should point back-right (-45)
        //rb (bottom-right) should point forward-right (135)
        assertTrue(states[0].speed > 0);
        assertTrue(states[1].speed > 0);
    }
}
