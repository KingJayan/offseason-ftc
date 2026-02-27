package org.firstinspires.ftc.teamcode.TeleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Swerve.Drivetrain;
import org.firstinspires.ftc.teamcode.Swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.Swerve.ModuleState;

/**verifies 3-wheeled hardware orientation*/
@TeleOp(name="swerve debug")
public class SwerveDebugOp extends OpMode {
    private Drivetrain dt;
    
    @Override
    public void init() {
        dt = new Drivetrain(hardwareMap);
    }

    @Override
    public void loop() {
        dt.update();
        
        // individual rotation tests
        if (gamepad1.a) dt.getL().set(new ModuleState(90, 0));
        else if (gamepad1.b) dt.getR().set(new ModuleState(90, 0));
        else if (gamepad1.x) dt.getB().set(new ModuleState(90, 0));
        else {
            dt.getL().set(new ModuleState(0, 0));
            dt.getR().set(new ModuleState(0, 0));
            dt.getB().set(new ModuleState(0, 0));
        }

        // drive motor tests
        double p = gamepad1.right_trigger;
        if (p > 0.1) {
            dt.getL().set(new ModuleState(0, p));
            dt.getR().set(new ModuleState(0, p));
            dt.getB().set(new ModuleState(0, p));
        }

        dt.drive(0,0,0,false);

        telemetry.addData("h", "%.1f", dt.getHeading());
        addMod("l", dt.getL()); addMod("r", dt.getR()); addMod("b", dt.getB());
        telemetry.update();
    }

    private void addMod(String n, SwerveModule m) {
        telemetry.addData(n, "cur:%.1f tgt:%.1f", m.getCurDeg(), m.getTgtDeg());
    }
}
