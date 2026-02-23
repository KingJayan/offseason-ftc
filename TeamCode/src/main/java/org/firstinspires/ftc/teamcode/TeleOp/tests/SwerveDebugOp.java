package org.firstinspires.ftc.teamcode.TeleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Swerve.Drivetrain;
import org.firstinspires.ftc.teamcode.Swerve.SwerveModule;

/**simple opmode to verify hardware and module orientation*/
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
        
        // test 1: individual module rotation
        // use a,b,x,y to rotate each module to 90 degrees
        if (gamepad1.a) dt.getLF().set(new org.firstinspires.ftc.teamcode.Swerve.ModuleState(90, 0));
        else if (gamepad1.b) dt.getRF().set(new org.firstinspires.ftc.teamcode.Swerve.ModuleState(90, 0));
        else if (gamepad1.x) dt.getLB().set(new org.firstinspires.ftc.teamcode.Swerve.ModuleState(90, 0));
        else if (gamepad1.y) dt.getRB().set(new org.firstinspires.ftc.teamcode.Swerve.ModuleState(90, 0));
        else {
            dt.getLF().set(new org.firstinspires.ftc.teamcode.Swerve.ModuleState(0, 0));
            dt.getRF().set(new org.firstinspires.ftc.teamcode.Swerve.ModuleState(0, 0));
            dt.getLB().set(new org.firstinspires.ftc.teamcode.Swerve.ModuleState(0, 0));
            dt.getRB().set(new org.firstinspires.ftc.teamcode.Swerve.ModuleState(0, 0));
        }

        // test 2: drive motor check
        // use right trigger to spin all drive motors forward
        double p = gamepad1.right_trigger;
        if (p > 0.1) {
            dt.getLF().set(new org.firstinspires.ftc.teamcode.Swerve.ModuleState(0, p));
            dt.getRF().set(new org.firstinspires.ftc.teamcode.Swerve.ModuleState(0, p));
            dt.getLB().set(new org.firstinspires.ftc.teamcode.Swerve.ModuleState(0, p));
            dt.getRB().set(new org.firstinspires.ftc.teamcode.Swerve.ModuleState(0, p));
        }

        dt.drive(0,0,0,false); // trigger execution logic

        telemetry.addData("heading", "%.1f", dt.getHeading());
        addModTelemetry("lf", dt.getLF());
        addModTelemetry("rf", dt.getRF());
        addModTelemetry("lb", dt.getLB());
        addModTelemetry("rb", dt.getRB());
        telemetry.update();
    }

    private void addModTelemetry(String name, SwerveModule m) {
        telemetry.addData(name, "cur:%.1f tgt:%.1f", m.getCurA(), m.getTgtA());
    }
}
