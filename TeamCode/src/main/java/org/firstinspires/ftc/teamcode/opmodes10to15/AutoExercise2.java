package org.firstinspires.ftc.teamcode.opmodes10to15;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms10to15.ProgrammingBoard8;

@Autonomous
public class AutoExercise2 extends OpMode {
    ProgrammingBoard8 board = new ProgrammingBoard8();
    enum State {
        START,
        FIRST_STEP,
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
                if (getRuntime() >= 5 || board.getDistance(DistanceUnit.CM) < 5) {
                    state = State.DONE;
                    lastTime = getRuntime();
                }
            default:
                telemetry.addData("Auto", "Finished");
        }
    }
}
