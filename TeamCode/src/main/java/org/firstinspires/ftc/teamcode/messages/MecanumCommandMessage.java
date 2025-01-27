<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RoadRunner/message/MecanumCommandMessage.java
package org.firstinspires.ftc.teamcode.RoadRunner.message;
========
package org.firstinspires.ftc.teamcode.messages;
>>>>>>>> parent of 39c5237 (set up basic files):TeamCode/src/main/java/org/firstinspires/ftc/teamcode/messages/MecanumCommandMessage.java

public final class MecanumCommandMessage {
    public long timestamp;
    public double voltage;
    public double leftFrontPower;
    public double leftBackPower;
    public double rightBackPower;
    public double rightFrontPower;

    public MecanumCommandMessage(double voltage, double leftFrontPower, double leftBackPower, double rightBackPower, double rightFrontPower) {
        this.timestamp = System.nanoTime();
        this.voltage = voltage;
        this.leftFrontPower = leftFrontPower;
        this.leftBackPower = leftBackPower;
        this.rightBackPower = rightBackPower;
        this.rightFrontPower = rightFrontPower;
    }
}
