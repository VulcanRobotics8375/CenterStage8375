package org.firstinspires.ftc.teamcode.opmodes1to10;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms1to10.ProgrammingBoard5;

@TeleOp
public class ServoGamepadOpMode extends OpMode {
    ProgrammingBoard5 board = new ProgrammingBoard5();
    @Override
    public void init() {
        board.init(hardwareMap);
    }
    @Override
    public void loop() {

        double servoPosition = gamepad1.left_trigger;
        board.setServoPosition(servoPosition);

//        if (gamepad1.a) {
//            board.setServoPosition(1.0);
//        }
//        else if (gamepad1.b) {
//            board.setServoPosition(0.0);
//        }
//        else {
//            board.setServoPosition(0.5);
//        }
    }
}
