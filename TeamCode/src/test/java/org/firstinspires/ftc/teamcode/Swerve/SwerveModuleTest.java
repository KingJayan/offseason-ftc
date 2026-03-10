package org.firstinspires.ftc.teamcode.Swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.config.Constants;
import org.junit.jupiter.api.Test;

public class SwerveModuleTest {

    @Test
    public void testUpdateReadsAnalogAngle() {
        DcMotorEx drive = mock(DcMotorEx.class);
        CRServo steer = mock(CRServo.class);
        AnalogInput analog = mock(AnalogInput.class);
        when(analog.getVoltage()).thenReturn(Constants.STEER_ANALOG_MAX_V / 2.0);

        SwerveModule module = new SwerveModule(drive, steer, analog, false, false, 0.0);
        module.update();

        assertEquals(180.0, module.getCurDeg(), 0.01);
    }

    @Test
    public void testExecuteStopsSteerInsideJitterBand() {
        DcMotorEx drive = mock(DcMotorEx.class);
        CRServo steer = mock(CRServo.class);
        AnalogInput analog = mock(AnalogInput.class);
        when(analog.getVoltage()).thenReturn(0.0);

        SwerveModule module = new SwerveModule(drive, steer, analog, false, false, 0.0);
        module.update();
        module.set(new ModuleState(Constants.STEER_JITTER_DEG / 2.0, 0.0));
        module.execute(1.0);

        verify(steer, times(1)).setPower(0.0);
        verify(drive, times(1)).setPower(0.0);
    }

    @Test
    public void testExecuteTurnsAcrossWrapShortestDirection() {
        DcMotorEx drive = mock(DcMotorEx.class);
        CRServo steer = mock(CRServo.class);
        AnalogInput analog = mock(AnalogInput.class);
        when(analog.getVoltage()).thenReturn(Constants.STEER_ANALOG_MAX_V * (170.0 / 360.0));

        SwerveModule module = new SwerveModule(drive, steer, analog, false, false, 0.0);
        module.update();
        module.set(new ModuleState(-170.0, 0.0));
        module.execute(1.0);

        verify(steer, times(1)).setPower(anyDouble());
    }
}

