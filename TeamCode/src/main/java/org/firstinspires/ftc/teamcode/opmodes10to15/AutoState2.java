package org.firstinspires.ftc.teamcode.opmodes10to15;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms10to15.ProgrammingBoard8;

@Autonomous
public class AutoState2 extends OpMode {
    ProgrammingBoard8 board = new ProgrammingBoard8();
    int state;
    @Override
    public void init() {
        board.init(hardwareMap);
    }
    @Override
    public void start() {
        state = 0;
    }
    @Override
    public void loop() {
        telemetry.addData("State", state);
        switch (state) {
            case 0:
                board.setServoPosition(0.5);
                if (board.isTouchSensorPressed()) {
                    state = 1;
                }
                break;
            case 1:
                board.setServoPosition(0.0);
                if (!board.isTouchSensorPressed()) {
                    state = 2;
                }
                break;
            case 2:
                board.setServoPosition(1.0);
                board.setMotorSpeed(0.5);
                if (board.getPotAngle() > 90) {
                    state = 3;
                }
                break;
            case 3:
                board.setMotorSpeed(0.0);
                state = 4;
                break;
            default:
                telemetry.addData("Auto", "Finished");
        }
    }
}
