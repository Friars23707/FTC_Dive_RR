package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ClawTurnTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        double servoPos = 1;
        boolean canPress = true;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                if (canPress) {
                    servoPos -= 0.1;
                    canPress = false;
                }
            } else if (gamepad1.b) {
                if (canPress) {
                    servoPos += 0.1;
                    canPress = false;
                }
            } else {
                canPress = true;
            }

            wrist.setPosition(servoPos);
            telemetry.addData("ServoPos: ", servoPos);
            telemetry.update();

        }
    }
}
