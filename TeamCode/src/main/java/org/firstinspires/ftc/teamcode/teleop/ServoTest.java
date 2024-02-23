package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class ServoTest extends OpMode {

    Servo servo;
    double servoPos = 0.5;

    public void init() {
        servo = hardwareMap.servo.get("5");
    }

    public void loop () {
        servoPos = Range.clip(servoPos - 0.01*gamepad1.left_stick_y, 0.01, 0.99);
        servo.setPosition(servoPos);
        telemetry.addData("servo pos", servoPos);
    }
}
