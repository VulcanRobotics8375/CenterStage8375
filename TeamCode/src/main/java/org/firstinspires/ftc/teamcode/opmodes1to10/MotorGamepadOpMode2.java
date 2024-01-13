package org.firstinspires.ftc.teamcode.opmodes1to10;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms1to10.ProgrammingBoard4;

@TeleOp
public class MotorGamepadOpMode2 extends OpMode {
    ProgrammingBoard4 board = new ProgrammingBoard4();
    @Override
    public void init() {
        board.init(hardwareMap);
    }
    @Override
    public void loop() {
        double motorSpeed = gamepad1.left_stick_y;
        motorSpeed = squareInputWithSign(motorSpeed);
        board.setMotorSpeed(motorSpeed);
        if(gamepad1.a) {
            board.setMotorZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if(gamepad1.b) {
            board.setMotorZeroBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetry.addData("Motor speed", motorSpeed);
        telemetry.addData("Motor rotations", board.getMotorRotations());
    }
    double squareInputWithSign(double input) {
        double output = input * input;
        if (input < 0) {
            output = output * -1;
        }
        return output;
    }
}
