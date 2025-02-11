package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class FiveInchTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initPose = new Pose2d(0, 0, 0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, initPose);

        Action test = drive.actionBuilder(initPose)
                .splineToConstantHeading(new Vector2d(5, 0), 0)
                .splineToConstantHeading(new Vector2d(0, 0), 0)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("start", drive.pose);
            telemetry.update();
            Actions.runBlocking(
                    new SequentialAction(
                            test
                    )
            );
            telemetry.addData("end", drive.pose);
            telemetry.update();
        }

    }
}
