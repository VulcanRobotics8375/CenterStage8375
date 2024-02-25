package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Toggle;

@TeleOp
public class IntakeTest extends OpMode {

    CRServo intake1, intake2;
    Toggle intakeToggle = new Toggle();

    public void init() {
        intake1 = hardwareMap.crservo.get("intake1");
        intake2 = hardwareMap.crservo.get("intake2");
    }

    public void loop () {
        intake1.setPower(-1);
        intake2.setPower(1);
    }
}
