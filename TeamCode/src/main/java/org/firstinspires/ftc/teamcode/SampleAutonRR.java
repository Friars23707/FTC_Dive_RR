package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonClasses.Claw;
import org.firstinspires.ftc.teamcode.AutonClasses.Slide;

@Autonomous(group = "RR_Autos")
public class SampleAutonRR extends LinearOpMode {

    PinpointDrive drive;
    Claw claw;
    Slide slide;
    Pose2d initialPose = new Pose2d(0,0, 0);
    final double sampleY = -45;
    final double[] bucketPos = {41.5, -17.5, Math.toRadians(35)}; // I hate radians

    final double ejectionWait = 1.5;
    final double pickupWait = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new PinpointDrive(hardwareMap, initialPose);
        claw = new Claw(hardwareMap);
        slide = new Slide(hardwareMap);

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
                .splineToLinearHeading(new Pose2d(13, sampleY+6, 0), 0)
                .splineToLinearHeading(new Pose2d(13, sampleY, 0), 0)
                .stopAndAdd(claw.collect())
                .stopAndAdd(slide.collection(true))
                .splineToConstantHeading(new Vector2d(18, sampleY), 0)
                .waitSeconds(pickupWait)

                //EJECTION
                .stopAndAdd(slide.extend(false))
                .splineTo(new Vector2d(bucketPos[0]-2, bucketPos[1]-2), bucketPos[2]+Math.toRadians(15))
                .stopAndAdd(claw.eject())
                .waitSeconds(ejectionWait)

                //PICK UP 2
                .splineToLinearHeading(new Pose2d(24, sampleY - 1, 0), 0)
                .stopAndAdd(claw.collect())
                .stopAndAdd(slide.collection(true))
                .splineToConstantHeading(new Vector2d(38, sampleY), 0)
                .waitSeconds(pickupWait)
                .splineToConstantHeading(new Vector2d(30, sampleY), 0)

                //EJECTION
                .stopAndAdd(slide.extend(false))
                .splineTo(new Vector2d(bucketPos[0]-2, bucketPos[1]-2), bucketPos[2]+Math.toRadians(25))
                .stopAndAdd(claw.eject())
                .waitSeconds(ejectionWait)

                //PICK UP 3
                .splineToLinearHeading(new Pose2d(40, sampleY - 1, 0), 0)
                .stopAndAdd(claw.collect())
                .stopAndAdd(slide.collection(true))
                .splineToConstantHeading(new Vector2d(45, sampleY), 0)
                .waitSeconds(pickupWait)

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
