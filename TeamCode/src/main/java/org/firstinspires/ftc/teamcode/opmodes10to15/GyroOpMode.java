package org.firstinspires.ftc.teamcode.opmodes10to15;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms10to15.ProgrammingBoard8;

@TeleOp
public class GyroOpMode extends OpMode {
    ProgrammingBoard8 board = new ProgrammingBoard8();
    @Override
    public void init() {
        board.init(hardwareMap);
    }
    @Override
    public void loop() {
        telemetry.addData("Our Heading in Degrees", board.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Our Heading in Radians", board.getHeading(AngleUnit.RADIANS));
        if(board.getHeading(AngleUnit.RADIANS) == 0) {
            board.setMotorSpeed(0);
        }
        else if (board.getHeading(AngleUnit.RADIANS) < 0) {
            board.setMotorSpeed(-1);
        }
        else if (board.getHeading(AngleUnit.RADIANS) > 0) {
            board.setMotorSpeed(1);
        }
    }
}
