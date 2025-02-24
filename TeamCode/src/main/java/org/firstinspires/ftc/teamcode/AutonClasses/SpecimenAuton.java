package org.firstinspires.ftc.teamcode.AutonClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonClasses.Threaded.JavaCall;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(group = "RR_Autos")
public class SpecimenAuton extends LinearOpMode {

    PinpointDrive drive;
    Claw claw;
    JavaCall slide;
    Pose2d initialPose = new Pose2d(0,0, 0);
    final double sampleY = -35;
    final double[] bucketPos = {44.3, -8.5, Math.toRadians(45)}; // I hate radians

    final double ejectionWait = 1.5;
    final double pickupWait = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new PinpointDrive(hardwareMap, initialPose);
        claw = new Claw(hardwareMap);
        slide = new JavaCall(hardwareMap);

        telemetry.addData("Setting up Trajectories", true);
        telemetry.addData("DO NOT MOVE ROBOT", true);
        telemetry.update();

        Action MEMEMEMEME = drive.actionBuilder(initialPose)
                //DRIVE TO SUB
                .stopAndAdd(claw.collect())
                .stopAndAdd(slide.extend(false))
                .splineTo(new Vector2d(20,8),0)
                .stopAndAdd(claw.placeSpecimen(0.0))
                .stopAndAdd(slide.placeSpecimen(false))
                .stopAndAdd(claw.placeSpecimen(1))
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
