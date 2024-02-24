package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ExtendoTest extends OpMode {
    DcMotorEx extendoLeft, extendoRight;
    public void init() {
        extendoLeft = hardwareMap.get(DcMotorEx.class, "extendoLeft");
        extendoLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        extendoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extendoRight = hardwareMap.get(DcMotorEx.class, "extendoRight");
        extendoRight.setDirection(DcMotorSimple.Direction.FORWARD);
        extendoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop() {
        extendoLeft.setPower(-gamepad1.left_stick_y);
        telemetry.addData("lift left", extendoLeft.getCurrentPosition());

        extendoRight.setPower(-gamepad1.left_stick_y);
        telemetry.addData("lift right", extendoRight.getCurrentPosition());

    }
}
