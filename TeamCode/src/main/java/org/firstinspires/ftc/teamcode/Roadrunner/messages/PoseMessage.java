package org.firstinspires.ftc.teamcode.Roadrunner.messages;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public final class PoseMessage {
    public long timestamp;
    public double x;
    public double y;
    public double heading;

    public PoseMessage(Pose2D pose) {
        this.timestamp = System.nanoTime();
        this.x = pose.position.x;
        this.y = pose.position.y;
        this.heading = pose.heading.toDouble();
    }
}

