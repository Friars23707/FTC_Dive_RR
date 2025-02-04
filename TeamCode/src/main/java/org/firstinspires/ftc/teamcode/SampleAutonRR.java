package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AutonClasses.Claw;
import org.firstinspires.ftc.teamcode.AutonClasses.Slide;

@Autonomous(group = "RR_Autos")
public class SampleAutonRR extends LinearOpMode {

    PinpointDrive drive;
    Claw claw;
    Slide slide;
    Pose2d initialPose = new Pose2d(0,0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new PinpointDrive(hardwareMap, initialPose);
        claw = new Claw(hardwareMap);
        slide = new Slide(hardwareMap);

        telemetry.addData("Setting up Trajectories", true);
        telemetry.update();

        Action goToBucket = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(20, -3), -25)
                .build();

        telemetry.addData("Trajectories Set Up", true);
        telemetry.addData("DO NOT MOVE ROBOT", true);
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        slide.extend(false),
                        goToBucket
                )
        );


    }


}
