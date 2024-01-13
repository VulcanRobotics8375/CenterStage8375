package org.firstinspires.ftc.teamcode.opmodes1to10;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms1to10.ProgrammingBoard4;

@TeleOp
public class MotorSensorOpMode extends OpMode {
    ProgrammingBoard4 board = new ProgrammingBoard4();
    @Override
    public void init() {
        board.init(hardwareMap);
    }
    @Override
    public void loop() {
        if (board.isTouchSensorPressed()) {
            board.setMotorSpeed(0.5);
        }
        else {
            board.setMotorSpeed(0.0);
        }
        telemetry.addData("Motor rotations", board.getMotorRotations());
    }
}
