<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RoadRunner/message/ThreeDeadWheelInputsMessage.java
package org.firstinspires.ftc.teamcode.RoadRunner.message;
========
package org.firstinspires.ftc.teamcode.messages;
>>>>>>>> parent of 39c5237 (set up basic files):TeamCode/src/main/java/org/firstinspires/ftc/teamcode/messages/ThreeDeadWheelInputsMessage.java

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

public final class ThreeDeadWheelInputsMessage {
    public long timestamp;
    public PositionVelocityPair par0;
    public PositionVelocityPair par1;
    public PositionVelocityPair perp;

    public ThreeDeadWheelInputsMessage(PositionVelocityPair par0, PositionVelocityPair par1, PositionVelocityPair perp) {
        this.timestamp = System.nanoTime();
        this.par0 = par0;
        this.par1 = par1;
        this.perp = perp;
    }
}