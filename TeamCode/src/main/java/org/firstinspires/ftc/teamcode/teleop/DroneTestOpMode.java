package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@TeleOp(name = "DroneTestOpMode")
public class DroneTestOpMode extends OpModePipeline {
    MainConfig subsystems = new MainConfig();

    @Override
    public void init() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        super.init();
    }

    public void loop() {
        Robot.update();

        subsystems.intake.intake();
//        subsystems.hopper.testLeft(-gamepad2.right_stick_y);
        subsystems.droneLauncher.controlAngle(-gamepad2.left_stick_y);
        subsystems.droneLauncher.controlTrigger(-gamepad2.right_stick_y);
        subsystems.drivetrain.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


        telemetry.update();
    }
}