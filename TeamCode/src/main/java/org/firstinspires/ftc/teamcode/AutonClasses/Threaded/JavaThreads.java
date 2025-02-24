package org.firstinspires.ftc.teamcode.AutonClasses.Threaded;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class JavaThreads extends Thread {

    public HardwareMap hwM;
    public DcMotor leftArm;
    public DcMotor rightArm;
    public DcMotor slide;

    public Servo claw;
    public Servo wrist;

    public JavaThreads(HardwareMap sent_hwM) {
        hwM = sent_hwM;

        leftArm = hwM.get(DcMotor.class,"arm_left");
        rightArm = hwM.get(DcMotor.class,"arm_right");
        slide = hwM.get(DcMotor.class,"slide");

        leftArm.setDirection(DcMotor.Direction.REVERSE);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setDirection(DcMotor.Direction.FORWARD);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = hwM.get(Servo.class, "claw");
        wrist = hwM.get(Servo.class, "wrist");

        wrist.setPosition(0.9); //WRIST IN

        moveArm(20, 0);
    }

    public int armPos;
    public int slidePos;
    public double clawPos;
    public  double clawSpeed;
    public boolean shouldWait;

    boolean latestArm = true;

    public void setArm(int armP, int slideP, boolean wait) {
        armPos = armP;
        slidePos = slideP;
        shouldWait = wait;

        latestArm = true;
    }

    public void setClaw(double clawP, double clawSpe) {
        clawPos = clawP;
        clawSpeed = clawSpe;

        latestArm = false;
    }

    private double lastArmPos = 0;

    public void moveArm(double armPos, double slidePos) {

        double armSpeed = lastArmPos <= armPos ? 0.7 : 0.5; //Fast up, slow down
        double slideSpeed = lastArmPos <= armPos ? 0.6 : 1; //slow up, fast down
        lastArmPos = armPos;

        leftArm.setTargetPosition((int) armPos);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm.setPower(armSpeed);

        rightArm.setTargetPosition((int) armPos);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setPower(armSpeed);

        slide.setTargetPosition((int) slidePos);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(slideSpeed);
    }

    public void run() {
        if (latestArm) {

            int savedArmPos = armPos;
            int savedSlidePos = slidePos;

            if (shouldWait) {
                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            moveArm(savedArmPos, savedSlidePos);

        } else {

            wrist.setPosition(clawPos);
            claw.setPosition(clawSpeed);

        }
    }

}