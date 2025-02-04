package org.firstinspires.ftc.teamcode.AutonClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Funcs extends LinearOpMode {
    public Action sleep(double time) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                sleep(time);
                return false;
            }
        };
    }


    @Override
    public void runOpMode() throws InterruptedException {
        //gives me the correct function
    }
}
