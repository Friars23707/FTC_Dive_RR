package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class TestOdo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initalPose = new Pose2d(0, 0, Math.toRadians(0));

        PinpointDrive drive = new PinpointDrive(hardwareMap, initalPose);

        Action forward = drive.actionBuilder(initalPose)
                .strafeTo(new Vector2d(5,0))
                .build();

        Action backwards = drive.actionBuilder(initalPose)
                .turn(Math.toRadians(90))
                .build();

        telemetry.addData("all built", true);
        telemetry.update();

        waitForStart();

        telemetry.addData("start", drive.pose);
        telemetry.update();
        Actions.runBlocking(
                new SequentialAction(
                        backwards
                )
        );
        telemetry.addData("end", drive.pose);
        telemetry.update();

    }
}
