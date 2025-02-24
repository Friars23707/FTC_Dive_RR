package org.firstinspires.ftc.teamcode.AutonClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;

/*
*
* NOT USED ANYMORE
* GO USE THE JAVACALL CLASS
* SO WE CAN MULTITHREAD YAYYYYYY
* LOGAN REMBISZ - FEBUARY 24TH 2025
*
* */

public class Slide extends Thread {

    public HardwareMap hwM;
    public DcMotor leftArm;
    public DcMotor rightArm;
    public DcMotor slide;
    Map<String, Integer[]> slideValues = new HashMap<>();

    public Slide(HardwareMap sent_hwM) {
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

        move(20, 0);

        // Adding key-value pairs to the map
        slideValues.put("collect", new Integer[]{20, -70});
        slideValues.put("extend", new Integer[]{1475, -2100});
    }

    public Action extend(boolean shouldWait) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (shouldWait) {
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
                move(1475, -2100);
                return false;
            }
        };
    }

    public Action collection(boolean shouldWait) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (shouldWait) {
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
                move(20, -70);
                return false;
            }
        };
    }

    public Action level1(boolean shouldWait) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (shouldWait) {
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
                move(1000, -1200);
                return false;
            }
        };
    }

    public Action retract(boolean shouldWait) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (shouldWait) {
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
                move(20, 0);
                return false;
            }
        };
    }

    public Action highRung(boolean shouldWait) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                move(1300, -1600);
                if (shouldWait) {
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
                return false;
            }
        };
    }

    public Action highRungBack(boolean shouldWait) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                move(600, -800);
                if (shouldWait) {
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
                return false;
            }
        };
    }

    private int lastArmPos = 0;

    public void move(int armPos, int slidePos) {
        double armSpeed = lastArmPos <= armPos ? 0.7 : 0.5; //Fast up, slow down
        double slideSpeed = lastArmPos <= armPos ? 0.6 : 1; //slow up, fast down
        lastArmPos = armPos;

        leftArm.setTargetPosition(armPos);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm.setPower(armSpeed);
        
        rightArm.setTargetPosition(armPos);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setPower(armSpeed);

        slide.setTargetPosition(slidePos);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(slideSpeed);
    }

    public void run(String key, boolean shoudldWait) {
        if (shoudldWait) {
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        Integer[] vals = slideValues.get(key);
        move(vals[0], vals[1]);
    }

}
