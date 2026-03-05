package org.firstinspires.ftc.teamcode.TeleOp.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.config.Constants;

/**motor encoder readout for all three drive motors*/
@TeleOp(name="swerve offset tuner")
public class SwerveOffsetTuner extends OpMode {
    private DcMotorEx l;
    private DcMotorEx r;
    private DcMotorEx b;

    @Override
    public void init() {
        l = hardwareMap.get(DcMotorEx.class, Constants.L_DRIVE);
        r = hardwareMap.get(DcMotorEx.class, Constants.R_DRIVE);
        b = hardwareMap.get(DcMotorEx.class, Constants.B_DRIVE);
    }

    @Override
    public void loop() {
        telemetry.addLine("motor encoder readout");
        show("l", l);
        show("r", r);
        show("b", b);
        telemetry.update();
    }

    private void show(String n, DcMotorEx m) {
        telemetry.addData(n, "pos:%d vel:%.2f", m.getCurrentPosition(), m.getVelocity());
    }
}
