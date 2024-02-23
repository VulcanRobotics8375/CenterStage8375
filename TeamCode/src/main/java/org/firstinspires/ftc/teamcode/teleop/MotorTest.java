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
        motor = hardwareMap.dcMotor.get("idk");
    }

    public void loop () {
        motor.setPower(0.1*gamepad1.left_stick_y);
    }
}
