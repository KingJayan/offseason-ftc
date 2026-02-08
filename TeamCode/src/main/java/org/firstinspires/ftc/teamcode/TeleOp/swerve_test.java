package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "swerve test")
public class swerve_test extends OpMode {

    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private Servo lfServo, lbServo, rfServo, rbServo;


    @Override public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        lfServo = hardwareMap.get(Servo.class, "lfServo");
        rfServo = hardwareMap.get(Servo.class, "rfServo");
        lbServo = hardwareMap.get(Servo.class, "lbServo");
        rbServo = hardwareMap.get(Servo.class, "rbServo");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override public void loop() {
        drive();
    }

    private void drive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (Math.abs(x) < 0.05) x = 0;
        if (Math.abs(y) < 0.05) y = 0;

        double speed = Math.sqrt(y * y + x * x);
        double angle = Math.atan2(y, x);

        setAll(angle,speed);
    }
    public void setAll(double angle, double speed) {

        double angleDeg = Math.toDegrees(angle);
        if(angleDeg < 0) angleDeg += 360;

        double servoPos = angleDeg / 360.0;

        lfServo.setPosition(servoPos);
        rfServo.setPosition(servoPos);
        lbServo.setPosition(servoPos);
        rbServo.setPosition(servoPos);

        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);
    }

}
