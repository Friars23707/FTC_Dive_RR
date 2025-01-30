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
                .strafeTo(new Vector2d(-5,0))
                .build();

        telemetry.addData("all built", true);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("start", true);
            telemetry.update();
            Actions.runBlocking(
                    new SequentialAction(
                            forward,
                            backwards
                    )
            );
            telemetry.addData("end", true);
            telemetry.update();
        }

    }
}
