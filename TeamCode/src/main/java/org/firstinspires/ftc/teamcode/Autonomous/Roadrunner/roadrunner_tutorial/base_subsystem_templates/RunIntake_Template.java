package org.firstinspires.ftc.teamcode.Autonomous.Roadrunner.roadrunner_tutorial.base_subsystem_templates;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RunIntake_Template {
    private final DcMotorEx intakeMotor;

    public RunIntake_Template(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    // --------------------- Intake In --------------------- \\
    public class IntakeIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeMotor.setPower(1.0); // full power in
            packet.put("intakeMode", "IN"); // telemetry
            return false; // keep running forever until interrupted
        }
    }
    public Action in() {
        return new IntakeIn();
    }

    // --------------------- Intake Idle --------------------- \\
    public class IntakeIdle implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeMotor.setPower(0.0); // stop
            packet.put("intakeMode", "IDLE"); // telemetry
            return false; // keep motor at idle until interrupted
        }
    }
    public Action idle() {
        return new IntakeIdle();
    }

    // --------------------- Intake Out --------------------- \\
    public class IntakeOut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeMotor.setPower(-1.0); // full power out
            packet.put("intakeMode", "OUT"); // telemetry
            return false; // keep running forever until interrupted
        }
    }
    public Action out() {
        return new IntakeOut();
    }
}
