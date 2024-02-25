package org.firstinspires.ftc.teamcode.teleop;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Toggle;

@TeleOp(name = "MainOpMode")
public class MainOpMode extends OpModePipeline {
    MainConfig subsystems = new MainConfig();

    private RobotState robotState = RobotState.INTAKE;
    private Toggle stateToggle = new Toggle();

    private double lastTime = 0.0;
    ElapsedTime timer = new ElapsedTime();
    double timed=0.0;

    @Override
    public void init() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        super.init();
    }

    public void loop() {
        Robot.update();

        switch (robotState) {
            case INTAKE:
                subsystems.intake();
                telemetry.addData("state", "INTAKE");
                Pair<Boolean,Boolean> breaks = subsystems.intake.getBreaks();
                if (!breaks.first&&!breaks.second && timed == 0.0) {
                    timed = timer.milliseconds();
                }
                if (timed > timer.milliseconds()+1000) {
                    timed = 0.0;
                    robotState = RobotState.TRANSFER;
                }
                break;
            case TRANSFER:
                subsystems.transfer();
                if (stateToggle.toggle(gamepad1.a)) {
                    robotState = RobotState.DEPOSIT;
                }
                telemetry.addData("state", "TRANSFER");
                break;
            case DEPOSIT:
                subsystems.deposit();
                if (stateToggle.toggle(gamepad1.a)) {
                    robotState = RobotState.INTAKE;
                }
                telemetry.addData("state", "DEPOSIT");
                break;
        }
        telemetry.update();
    }

    public enum RobotState {
        INTAKE,
        TRANSFER,
        DEPOSIT
    }
}
