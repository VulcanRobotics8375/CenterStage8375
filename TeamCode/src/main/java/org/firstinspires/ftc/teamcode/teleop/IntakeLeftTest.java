package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

//@Disabled
@TeleOp(name = "IntakeLeftTest")
public class IntakeLeftTest extends OpModePipeline {
    MainConfig subsystems = new MainConfig();

    @Override
    public void init() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        super.init();
    }

    public void loop() {
        Robot.update();

//        subsystems.intake.testLeft(-gamepad2.left_stick_y);
//        subsystems.hopper.testHopperInternalServos(gamepad2.left_stick_y, gamepad2.left_stick_x);
//        subsystems.lift.run((gamepad1.right_trigger > 0) ? gamepad1.right_trigger : -gamepad1.left_trigger);
//        subsystems.lift.run(-gamepad2.left_stick_y, gamepad2.dpad_down, gamepad2.dpad_up);

        subsystems.drivetrain.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//        subsystems.hopper.test(-gamepad2.left_stick_y, -gamepad2.right_stick_y);
//        subsystems.hopper.testArms(gamepad1.a, gamepad1.b);


        telemetry.update();
    }
}