package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class ServoTest extends OpMode {

    Servo servo;
    Servo servo1;
    double servoPos = 0.1;
    double servo1Pos = 0.879;
    AnalogInput v4barLeftEnc, v4barRightEnc;
    // left: 0.575, right:394

    public void init() {
        servo = hardwareMap.servo.get("iV4barRight");
        servo1 = hardwareMap.servo.get("iV4barLeft");
//        servo1 = hardwareMap.servo.get("iV4BarRight");
//        v4barLeftEnc = hardwareMap.analogInput.get("dV4BarLEnc");
    }

    public void loop () {
        servoPos = servoPos - 0.01*gamepad1.left_stick_y;
        servo1Pos = servo1Pos + 0.01*gamepad1.left_stick_y;
        servo.setPosition(servoPos);
        servo1.setPosition(servo1Pos);
        telemetry.addData("servo pos", servoPos);
        telemetry.addData("servo1 pos", servo1Pos);


//        servo1Pos = Range.clip(servo1Pos + 0.01*gamepad1.right_stick_y, 0.01, 0.99);
//        servo1.setPosition(servo1Pos);
//        telemetry.addData("servo1 pos", servo1Pos);

//        telemetry.addData("voltage", v4barLeftEnc.getVoltage());
    }
}
