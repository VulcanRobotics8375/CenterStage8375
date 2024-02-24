package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class LiftTest extends OpMode {
    DcMotorEx liftLeft, liftRight;
    public void init() {
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop() {
        liftLeft.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x / 3.0);
        telemetry.addData("lift left", liftLeft.getCurrentPosition());

        liftRight.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x / 3.0);
        telemetry.addData("lift right", liftRight.getCurrentPosition());

    }
}
