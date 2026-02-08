package org.firstinspires.ftc.teamcode.Autonomous.Roadrunner.roadrunner_tutorial.base_subsystem_templates;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servo_Template {
    private final Servo servo;

    public Servo_Template(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "servo");
    }

    //-------------------------------Position 1-------------------------------\\
    public class Servo_Position1 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            servo.setPosition(0);
            return false;
        }
    }
    public Action toPos1() {
        return new Servo_Position1();
    }

    //-------------------------------Position 2-------------------------------\\
    public class Servo_Position2 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            servo.setPosition(0.6);
            return false;
        }
    }
    public Action toPos2() {
        return new Servo_Position2();
    }
}

