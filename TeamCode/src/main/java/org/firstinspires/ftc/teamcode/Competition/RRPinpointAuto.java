package org.firstinspires.ftc.teamcode.Competition;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.Roadrunner.PinpointDrive;

@Autonomous(name = "RR Pinpoint Auto", group = "RoadRunner")


//@Disabled
public class RRPinpointAuto extends OpMode {


    // Create a org.firstinspires.ftc.teamcode.RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.


    int legNumber = 0;
    boolean isRedAlliance = true;
    boolean isLeftStart = true;
    double startingPause = 0;
    private PinpointDrive drive;
    private Pose2d initialPose;

    @Override
    public void init() {
        // instantiate the PinpointDrive at a particular pose.
        initialPose = new Pose2d(0, 0, Math.toRadians(0));
        drive = new PinpointDrive(hardwareMap, initialPose);
    }
    Action moveForward;
    Action moveBack;
    @Override
    public void start(){
        Action moveForward = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(5, 0, Math.toRadians(0)), Math.toRadians(0))
                .build();
        Action moveBack = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0))
                .build();
    }

    @Override
    public void loop(){
        new SequentialAction(
                moveForward,
                moveBack
        );
    }
}
