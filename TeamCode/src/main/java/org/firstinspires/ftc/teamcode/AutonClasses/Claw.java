package org.firstinspires.ftc.teamcode.AutonClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Claw extends LinearOpMode {

    //FINAL-ISH WRIST VALUES
    final public double WRIST_IN = 0.9;
    final double WRIST_OUT = 0.53;
    final double SPIN_IN = 0.0;
    final double SPIN_HOLD = 0.5;
    final double SPIN_OUT = 1;

    public HardwareMap hwM;
    public Servo claw;
    public Servo wrist;

    public Claw(HardwareMap sent_hwM) {
        hwM = sent_hwM;

        claw = hwM.get(Servo.class, "claw");
        wrist = hwM.get(Servo.class, "wrist");

        wrist.setPosition(WRIST_IN);
    }


    public Action reset() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wrist.setPosition(WRIST_IN);
                claw.setPosition(SPIN_HOLD);
                return false;
            }
        };
    }

    public Action collect() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wrist.setPosition(WRIST_OUT);
                claw.setPosition(SPIN_IN);
                return false;
            }
        };
    }

    public Action eject() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wrist.setPosition(WRIST_OUT);
                claw.setPosition(SPIN_OUT);
                return false;
            }
        };
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //I want the sleep function :sob:
    }
}
