package org.firstinspires.ftc.teamcode.Autonomous.Roadrunner.roadrunner_tutorial.base_subsystem_templates;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motor_Template {
    private final DcMotorEx motor;

    public Motor_Template(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    //-----------------------------Position 1--------------------------------------\\
    public class MotorEx_Position1 implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                motor.setPower(-0.8); //Motor Speed
                initialized = true;
            }

            double pos = motor.getCurrentPosition();
            packet.put("motorPos", pos);
            if (pos > 100.0) {
                return true;
            } else {
                motor.setPower(0.01); //negative gravity
                return false;
            }
        }
    }
    public Action toPos1(){
        return new MotorEx_Position1();
    }

    //----------------------------Position2----------------------------------\\
    public class Position2 implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                motor.setPower(0.8);
                initialized = true;
            }

            double pos = motor.getCurrentPosition();
            packet.put("motorPos", pos);
            if (pos < 395 || pos > 405) {
                return true;
            } else {
                motor.setPower(0.01); //negative gravity
                return false;
            }
        }
    }
    public Action toPos2() {
        return new Position2();
    }

    //--------------------------Position 3---------------------------------\\
    public class Position3 implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                motor.setPower(0.8);
                initialized = true;
            }

            double pos = motor.getCurrentPosition();
            packet.put("motorPos", pos);
            if (pos < 2000) {
                return true;
            } else {
                motor.setPower(0.01); //negative gravity
                return false;
            }
        }
    }
    public Action toPos3() {
        return new Position3();
    }

}
