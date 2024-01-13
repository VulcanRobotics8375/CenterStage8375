package org.firstinspires.ftc.teamcode.opmodes1to10;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms1to10.ProgrammingBoard2;

@TeleOp
public class TouchSensorOpMode2 extends OpMode {
    ProgrammingBoard2 board = new ProgrammingBoard2();
    @Override
    public void init() {
        board.init(hardwareMap);
    }
    @Override
    public void loop() {
        telemetry.addData("Touch sensor", board.getTouchSensorState() ? "Pressed" : "Not pressed");
        telemetry.addData("Touch sensor released: ", board.isTouchSensorReleased());
    }
}
