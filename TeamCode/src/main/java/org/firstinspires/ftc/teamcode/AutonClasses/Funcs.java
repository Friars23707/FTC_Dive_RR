package org.firstinspires.ftc.teamcode.AutonClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Funcs extends LinearOpMode {
    private long tim;

    public Action wSleep(long ti) {
        tim = ti;
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                sleep(tim);
                return false;
            }
        };
    }


    @Override
    public void runOpMode() throws InterruptedException {
        //gives me the correct function
    }
}
