package org.firstinspires.ftc.teamcode.opmodes10to15;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms10to15.ProgrammingBoard7;

@TeleOp
public class DistanceColorOpMode extends OpMode {
    ProgrammingBoard7 board = new ProgrammingBoard7();
    @Override
    public void init() {
        board.init(hardwareMap);
    }
    @Override
    public void loop() {
        telemetry.addData("Amount red", board.getAmountRed());
        telemetry.addData("Amount blue", board.getAmountBlue());
        telemetry.addData("Distance (CM)", board.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance (IN)", board.getDistance(DistanceUnit.INCH));
        if(board.getDistance(DistanceUnit.CM) < 10) {
            board.setMotorSpeed(0);
        }
        else {
            board.setMotorSpeed(0.5);
        }
    }
}
