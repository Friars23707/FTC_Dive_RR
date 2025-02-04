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
import org.firstinspires.ftc.teamcode.AutonClasses.Funcs;
import org.firstinspires.ftc.teamcode.AutonClasses.Slide;

@Autonomous(group = "RR_Autos")
public class SampleAutonRR extends LinearOpMode {

    PinpointDrive drive;
    Claw claw;
    Slide slide;
    Funcs Funcs;
    Pose2d initialPose = new Pose2d(0,0, 0);
    final double sampleY = -37;

    final double ejectionWait = 500.0;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new PinpointDrive(hardwareMap, initialPose);
        claw = new Claw(hardwareMap);
        slide = new Slide(hardwareMap);
        Funcs = new Funcs();

        telemetry.addData("Setting up Trajectories", true);
        telemetry.addData("DO NOT MOVE ROBOT", true);
        telemetry.update();

        Action goToBucket = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(42, -5), Math.toRadians(35))
                .build();


        Action firstSampleLineUp = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(25, sampleY), 0)
                .build();

        Action firstSampleCollect = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(33, sampleY), 0)
                .build();


        Action secondSampleLineUp = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(33, sampleY), 0)
                .build();

        Action secondSampleCollect = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(41, sampleY), 0)
                .build();


        Action thirdSampleLineUp = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(41, sampleY), 0)
                .build();

        Action thirdSampleCollect = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(50, sampleY), 0)
                .build();

        telemetry.addData("Trajectories Set Up", true);
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        claw.collect(),
                        slide.extend(false),
                        goToBucket,
                        claw.eject(),
                        Funcs.sleep(ejectionWait),
                        slide.collection(false),

                        firstSampleLineUp,
                        claw.collect(),
                        firstSampleCollect,
                        slide.extend(false),
                        goToBucket,
                        claw.eject(),
                        Funcs.sleep(ejectionWait),
                        slide.collection(false),

                        secondSampleLineUp,
                        claw.collect(),
                        secondSampleCollect,
                        slide.extend(false),
                        goToBucket,
                        claw.eject(),
                        Funcs.sleep(ejectionWait),
                        slide.collection(false),

                        thirdSampleLineUp,
                        claw.collect(),
                        thirdSampleCollect,
                        slide.extend(false),
                        goToBucket,
                        claw.eject(),
                        Funcs.sleep(ejectionWait),
                        slide.collection(false)

                )
        );


    }


}
