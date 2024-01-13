package org.firstinspires.ftc.teamcode.opmodes10to15;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms10to15.ProgrammingBoard8;

@Autonomous
public class AutoTime extends OpMode {
    ProgrammingBoard8 board = new ProgrammingBoard8();
    enum State {
        START,
        SECOND_STEP,
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
                if (getRuntime() >= 3.0) {
                    state = State.SECOND_STEP;
                    lastTime = getRuntime();
                }
            case SECOND_STEP:
                if (getRuntime() >= lastTime + 3.0) {
                    state = State.DONE;
                    lastTime = getRuntime();
                }
                break;
            default:
                telemetry.addData("Auto", "Finished");
        }
    }
}
