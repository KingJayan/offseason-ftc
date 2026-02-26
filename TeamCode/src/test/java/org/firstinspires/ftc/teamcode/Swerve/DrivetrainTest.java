package org.firstinspires.ftc.teamcode.Swerve;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.config.Constants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.Collections;
import java.util.Iterator;

public class DrivetrainTest {
    private HardwareMap hw;
    private DcMotorEx drv;
    private CRServo str;
    private AnalogInput enc;
    private IMU imu;
    private VoltageSensor vs;
    private Drivetrain dt;

    @BeforeEach
    @SuppressWarnings("unchecked")
    public void setup() {
        drv = mock(DcMotorEx.class);
        str = mock(CRServo.class);
        enc = mock(AnalogInput.class);
        imu = mock(IMU.class);
        vs = mock(VoltageSensor.class);
        hw = mock(HardwareMap.class);

        when(hw.get(eq(DcMotorEx.class), anyString())).thenReturn(drv);
        when(hw.get(eq(CRServo.class), anyString())).thenReturn(str);
        when(hw.get(eq(AnalogInput.class), anyString())).thenReturn(enc);
        when(hw.get(eq(IMU.class), anyString())).thenReturn(imu);

        HardwareMap.DeviceMapping<VoltageSensor> vMap = mock(HardwareMap.DeviceMapping.class);
        Iterator<VoltageSensor> it = Collections.singleton(vs).iterator();
        doReturn(it).when(vMap).iterator();
        hw.voltageSensor = vMap;

        when(vs.getVoltage()).thenReturn(12.0);
        YawPitchRollAngles a = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);
        when(imu.getRobotYawPitchRollAngles()).thenReturn(a);

        dt = new Drivetrain(hw);
    }

//    @Test
//    public void testDefense() {
//        dt.defense();
//        verify(str, times(4)).setPower(anyDouble());
//    }
//
//    @Test
//    public void testStop() {
//        dt.stop();
//        verify(drv, times(4)).setPower(0.0);
//        verify(str, times(4)).setPower(0.0);
//    }

//    @Test
//    public void testUpdate() {
//        //verify update calls dont crash and reach sensors
//        dt.update();
//        verify(enc, times(4)).getVoltage();
//    }

    @Test
    public void testDriveRobCentric() {
        //test basic movement
        dt.drive(0.5, 0.5, 0.0, false);
        verify(drv, times(4)).setPower(anyDouble());
        verify(str, times(4)).setPower(anyDouble());
    }

    @Test
    public void testDriveFieldCentric() {
        //test movement with imu
        dt.drive(0.5, 0.0, 0.0, true);
        verify(imu, times(1)).getRobotYawPitchRollAngles();
        verify(drv, times(4)).setPower(anyDouble());
    }

//    @Test
//    public void testDriveDeadband() {
//        //test small inputs are ignored
//        dt.drive(0.001, 0.001, 0.001, false);
//        verify(drv, times(4)).setPower(0.0);
//    }
//
//    @Test
//    public void testResetYaw() {
//        dt.resetYaw();
//        verify(imu, times(1)).resetYaw();
//    }
}
