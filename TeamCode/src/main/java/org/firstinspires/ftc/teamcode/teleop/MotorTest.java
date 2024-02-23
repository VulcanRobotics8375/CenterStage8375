package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MotorTest extends OpMode {

    DcMotor motor;
    double servoPos = 0.5;

    public void init() {
        servo = hardwareMap.servo.get("idk");
    }

    public void loop () {
        servoPos -= 0.01*gamepad1.left_stick_y;
        servo.setPosition(servoPos);
        telemetry.addData("servo pos", servoPos);
    }
}
