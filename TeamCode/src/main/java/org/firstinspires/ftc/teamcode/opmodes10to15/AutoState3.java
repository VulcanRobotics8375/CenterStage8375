package org.firstinspires.ftc.teamcode.opmodes10to15;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms10to15.ProgrammingBoard8;

@Autonomous
public class AutoState3 extends OpMode {
    ProgrammingBoard8 board = new ProgrammingBoard8();
    String state = "START";
    @Override
    public void init() {
        board.init(hardwareMap);
    }
    @Override
    public void start() {
        state = "START";
    }
    @Override
    public void loop() {
        telemetry.addData("State", state);
        switch (state) {
            case "START":
                board.setServoPosition(0.5);
                if (board.isTouchSensorPressed()) {
                    state = "WAIT_FOR_SENSOR_RELEASE";
                }
                break;
            case "WAIT_FOR_SENSOR_RELEASE":
                board.setServoPosition(0.0);
                if (!board.isTouchSensorPressed()) {
                    state = "WAIT_FOR_POT_TURN";
                }
                break;
            case "WAIT_FOR_POT_TURN":
                board.setServoPosition(1.0);
                board.setMotorSpeed(0.5);
                if (board.getPotAngle() > 90) {
                    state = "STOP";
                }
                break;
            case "STOP":
                board.setMotorSpeed(0.0);
                state = "DONE";
                break;
            default:
                telemetry.addData("Auto", "Finished");
        }
    }
}
