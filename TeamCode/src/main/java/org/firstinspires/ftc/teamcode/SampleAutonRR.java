package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonClasses.Claw;
import org.firstinspires.ftc.teamcode.AutonClasses.Slide;
import org.firstinspires.ftc.teamcode.AutonClasses.Threaded.JavaCall;

import kotlin.jvm.internal.TypeParameterReference;

@Autonomous(group = "RR_Autos")
public class SampleAutonRR extends LinearOpMode {

    PinpointDrive drive;
    Claw claw;
    JavaCall slide;
    Pose2d initialPose = new Pose2d(0,0, 0);
    final double sampleY = -34.75;
    final double[] bucketPos = {45, -8, Math.toRadians(45)}; // I hate radians

    final double ejectionWait = 1.5;
    final double pickupWait = 0.9;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new PinpointDrive(hardwareMap, initialPose);
        claw = new Claw(hardwareMap);
        slide = new JavaCall(hardwareMap);

        telemetry.addData("Setting up Trajectories", true);
        telemetry.addData("DO NOT MOVE ROBOT", true);
        telemetry.update();

        Action MEMEMEMEME = drive.actionBuilder(initialPose)
                //INITIAL EJECTION
                .stopAndAdd(claw.collect())
                .stopAndAdd(slide.extend(false))
                .splineTo(new Vector2d(bucketPos[0], bucketPos[1]), bucketPos[2])
                .stopAndAdd(claw.eject())
                .waitSeconds(ejectionWait)

                //PICK UP 1
                .stopAndAdd(claw.collect())
                .stopAndAdd(slide.collection(true))
                .splineToLinearHeading(new Pose2d(32.5, sampleY+12, -45), -45)
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(35.5, sampleY+1), -45)
                .waitSeconds(pickupWait)
                .splineToConstantHeading(new Vector2d(30, sampleY+1), 0)
                // FORTNITE

                //EJECTION
                .stopAndAdd(slide.extend(false))
                .splineTo(new Vector2d(bucketPos[0], bucketPos[1]), bucketPos[2])
                .stopAndAdd(claw.eject())
                .waitSeconds(ejectionWait)

                //PICK UP 2
                .stopAndAdd(claw.collect())
                .stopAndAdd(slide.collection(true))
                .splineToLinearHeading(new Pose2d(34, sampleY, 0), 0)
                .splineToConstantHeading(new Vector2d(41, sampleY+1), 0)
                .waitSeconds(pickupWait)
                .splineToConstantHeading(new Vector2d(30, sampleY+1), 0)

                //EJECTION
                .stopAndAdd(slide.extend(false))
                .splineTo(new Vector2d(bucketPos[0], bucketPos[1]), bucketPos[2])
                .stopAndAdd(claw.eject())
                .waitSeconds(ejectionWait)

                //PICK UP 3
                .stopAndAdd(claw.collect())
                .stopAndAdd(slide.collection(true))
                .splineToLinearHeading(new Pose2d(45, sampleY, 0), 0)
                .splineToConstantHeading(new Vector2d(50, sampleY), 0)
                .waitSeconds(pickupWait)

                //EJECTION
                .stopAndAdd(slide.extend(true))
                .splineTo(new Vector2d(bucketPos[0], bucketPos[1]), bucketPos[2])
                .stopAndAdd(claw.eject())
                .waitSeconds(ejectionWait)

                /*ATTEMPT PARK
                .stopAndAdd(slide.level1(false))
                .splineToLinearHeading(new Pose2d(40, -40,180),0)
                */
                .build();


        telemetry.addData("Trajectories Set Up", true);
        telemetry.addData("ra", bucketPos[2]);
        telemetry.update();



        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        MEMEMEMEME
                        /*claw.collect(),
                        slide.extend(false),
                        goToBucket,
                        claw.eject(),
                        Funcs.sleep(ejectionWait),

                        firstSampleLineUp,
                        slide.collection(true),
                        claw.collect(),
                        firstSampleCollect,
                        slide.extend(false),
                        goToBucket,
                        claw.eject(),
                        Funcs.sleep(ejectionWait),

                        secondSampleLineUp,
                        slide.collection(true),
                        claw.collect(),
                        secondSampleCollect,
                        slide.extend(false),
                        goToBucket,
                        claw.eject(),
                        Funcs.sleep(ejectionWait),

                        thirdSampleLineUp,
                        slide.collection(true),
                        claw.collect(),
                        thirdSampleCollect,
                        slide.extend(false),
                        goToBucket,
                        claw.eject(),
                        Funcs.sleep(ejectionWait)
*/
                )
        );


    }


}
