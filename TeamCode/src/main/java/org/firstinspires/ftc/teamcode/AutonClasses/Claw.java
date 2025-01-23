package org.firstinspires.ftc.teamcode.AutonClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Claw extends LinearOpMode {

    public HardwareMap hwM;
    public Servo claw;
    public Servo wrist;

    public Claw(HardwareMap sent_hwM) {
        hwM = sent_hwM;

        claw = hwM.get(Servo.class, "claw");
        wrist = hwM.get(Servo.class, "wrist");

        wrist.setPosition(0.84);
    }

    public void reset() {
        wrist.setPosition(0.54);
        claw.setPosition(0.5);
    }

    public void collect() {

        wrist.setPosition(0.54);
        claw.setPosition(0.0);

    }

    public void eject() {

        wrist.setPosition(0.54);
        claw.setPosition(0.7);

    }
    public void side() {

        wrist.setPosition(0.12);
        claw.setPosition(0.5);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        //I want the sleep function :sob:
    }
}