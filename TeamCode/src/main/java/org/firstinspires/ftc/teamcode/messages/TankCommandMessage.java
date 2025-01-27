<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RoadRunner/message/TankCommandMessage.java
package org.firstinspires.ftc.teamcode.RoadRunner.message;
========
package org.firstinspires.ftc.teamcode.messages;
>>>>>>>> parent of 39c5237 (set up basic files):TeamCode/src/main/java/org/firstinspires/ftc/teamcode/messages/TankCommandMessage.java

public final class TankCommandMessage {
    public long timestamp;
    public double voltage;
    public double leftPower;
    public double rightPower;

    public TankCommandMessage(double voltage, double leftPower, double rightPower) {
        this.timestamp = System.nanoTime();
        this.voltage = voltage;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
    }
}
