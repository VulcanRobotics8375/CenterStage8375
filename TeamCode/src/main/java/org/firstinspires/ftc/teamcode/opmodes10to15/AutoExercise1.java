package org.firstinspires.ftc.teamcode.opmodes10to15;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms10to15.ProgrammingBoard8;

@Autonomous
public class AutoExercise1 extends OpMode {
    ProgrammingBoard8 board = new ProgrammingBoard8();
    enum State {
        START,
        FIRST_STEP,
        SECOND_STEP,
        THIRD_STEP,

        DONE
    }
    State state = State.START;
    double lastTime;
    @Override
    public void init() {
        board.init(hardwareMap);
    }
    @Override
    public void start() {
        state = State.START;
        resetRuntime();
        lastTime = getRuntime();
    }
    @Override
    public void loop() {
        telemetry.addData("State", state);
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Time in State", getRuntime() - lastTime);
        switch (state) {
            case START:
                board.setMotorSpeed(0.25);
                if (getRuntime() >= .250) {
                    state = State.FIRST_STEP;
                    lastTime = getRuntime();
                }
            case FIRST_STEP:
                board.setMotorSpeed(0.50);
                if (getRuntime() >= lastTime + 0.25) {
                    state = State.SECOND_STEP;
                    lastTime = getRuntime();
                }
                break;
            case SECOND_STEP:
                board.setMotorSpeed(0.75);
                if (getRuntime() >= lastTime + 0.25) {
                    state = State.THIRD_STEP;
                    lastTime = getRuntime();
                }
            case THIRD_STEP:
                board.setMotorSpeed(1.0);
                if (board.isTouchSensorPressed()) {
                    state = State.DONE;
                }
            default:
                telemetry.addData("Auto", "Finished");
        }
    }
}
