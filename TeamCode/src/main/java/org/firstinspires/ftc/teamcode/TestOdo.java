package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonClasses.Claw;

@Autonomous
@SuppressWarnings("unused")
public class TestOdo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        Claw claw = new Claw(hardwareMap);

        Action first = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(20,0), 0)
                .stopAndAdd(claw.collect())
                .splineTo(new Vector2d(-20, -20), 0)
                .build();
        Action second = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-20, -20), 0)
                .build();

        telemetry.addData("all built", true);
        telemetry.update();

        waitForStart();

        telemetry.addData("first", drive.pose);
        telemetry.update();

        Actions.runBlocking(new SequentialAction(first));

        telemetry.addData("first end", drive.pose);
        telemetry.update();

        sleep(5000);

        telemetry.addData("second", drive.pose);
        telemetry.update();

        Actions.runBlocking(new SequentialAction(second));

        telemetry.addData("second end", drive.pose);
        telemetry.update();
    }
}
