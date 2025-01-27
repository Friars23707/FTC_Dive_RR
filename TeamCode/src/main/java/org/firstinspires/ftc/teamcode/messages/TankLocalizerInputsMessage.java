<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RoadRunner/message/TankLocalizerInputsMessage.java
package org.firstinspires.ftc.teamcode.RoadRunner.message;
========
package org.firstinspires.ftc.teamcode.messages;
>>>>>>>> parent of 39c5237 (set up basic files):TeamCode/src/main/java/org/firstinspires/ftc/teamcode/messages/TankLocalizerInputsMessage.java

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

import java.util.List;

public final class TankLocalizerInputsMessage {
    public long timestamp;
    public PositionVelocityPair[] left;
    public PositionVelocityPair[] right;

    public TankLocalizerInputsMessage(List<PositionVelocityPair> left, List<PositionVelocityPair> right) {
        this.timestamp = System.nanoTime();
        this.left = left.toArray(new PositionVelocityPair[0]);
        this.right = right.toArray(new PositionVelocityPair[0]);
    }
}
