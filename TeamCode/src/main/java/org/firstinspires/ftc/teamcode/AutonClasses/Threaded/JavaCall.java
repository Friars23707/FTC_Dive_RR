package org.firstinspires.ftc.teamcode.AutonClasses.Threaded;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;

public class JavaCall {

    JavaThreads threads;

    public JavaCall(HardwareMap sent_hwM) {
        threads = new JavaThreads(sent_hwM);
    }

    public Action extend(boolean shouldWait) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                threads.setArm(1500, -2000, shouldWait);
                threads.start();

                return false;
            }
        };
    }

    public Action collection(boolean shouldWait) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                threads.setArm(0, -500, shouldWait);
                threads.start();

                return false;
            }
        };
    }

    public Action level1(boolean shouldWait) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                threads.setArm(1000, -1200, shouldWait);
                threads.start();

                return false;
            }
        };
    }


    public Action placeSpecimen(boolean shouldWait,int armP) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                threads.setArm(armP, -1300, shouldWait);
                threads.start();

                return false;
            }
        };
    }

    public Action retractSpecimen(boolean shouldWait) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                threads.setArm(900, -1000, shouldWait);
                threads.start();

                return false;
            }
        };
    }

}