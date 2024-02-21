package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
                if(subsystems.intake.intakingComplete() || stateToggle.toggle(gamepad1.a)) {
                    robotState = RobotState.TRANSFER;
                    subsystems.intake.resetBreakBeam();
                }
                telemetry.addData("state", "INTAKE");
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
