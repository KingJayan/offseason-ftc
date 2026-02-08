

package org.firstinspires.ftc.teamcode.Autonomous.Encoder_Drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class EncoderAutonTemplate extends LinearOpMode {
    //Declare Motors
    //private double tick = 45.2763689698;
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;

    @Override
    public void runOpMode() {

        //---------------- INITIALIZATION ----------------\\
        //Drive Motor Config
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        //Resets all encoder values to 0
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //Sets motor target positions to 0
        leftFront.setTargetPosition(0);
        rightFront.setTargetPosition(0);
        leftBack.setTargetPosition(0);
        rightBack.setTargetPosition(0);

        //Set RunMode to Run to Position
        leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //When power of 0 is applied, it will brake, rather than glide with inertia
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        //TODO: Set motor directions here, when a positive power is applied, it should go forward
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        //--------------------- RUN ---------------------\\
        //waits for the Driver Hub to receive "start" input

        waitForStart();

        //TODO: Pathing, use the functions below to move the robot

    }

    //Positive value will go forward, Negative value will go backward
    public void vertical(int ticks, double speed, long seconds) {
        reset();

        leftFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks);
        rightFront.setTargetPosition(ticks);
        rightBack.setTargetPosition(ticks);

        leftFront.setVelocity(speed);
        leftBack.setVelocity(speed);
        rightFront.setVelocity(speed);
        rightBack.setVelocity(speed);

        sleep(seconds * 1000);
    }

    //Positive value will strafe right, Negative value will strafe left
    public void strafe(int ticks, double speed, long seconds) {
        reset();

        leftFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(-ticks);
        rightFront.setTargetPosition(-ticks);
        rightBack.setTargetPosition(ticks);

        leftFront.setVelocity(speed);
        leftBack.setVelocity(speed);
        rightFront.setVelocity(speed);
        rightBack.setVelocity(speed);

        sleep(seconds * 1000);
    }

    //Positive value will turn right, Negative value will turn left
    public void turn(int ticks, double speed, long seconds) {
        reset();

        leftFront.setTargetPosition(-ticks);
        leftBack.setTargetPosition(-ticks);
        rightFront.setTargetPosition(ticks);
        rightBack.setTargetPosition(ticks);

        leftFront.setVelocity(speed);
        leftBack.setVelocity(speed);
        rightFront.setVelocity(speed);
        rightBack.setVelocity(speed);

        sleep(seconds * 1000);
    }

    public void reset() {
        //Resets all encoder values to 0
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //Sets target position to 0
        leftFront.setTargetPosition(0);
        rightFront.setTargetPosition(0);
        leftBack.setTargetPosition(0);
        rightBack.setTargetPosition(0);

        //Sets motor mode to Run to Position to make sure no motor has a target position set (0)
        leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

}