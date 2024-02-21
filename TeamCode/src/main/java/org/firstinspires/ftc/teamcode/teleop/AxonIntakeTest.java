package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

//@Disabled
@TeleOp(name = "AxonIntakeTest")
public class AxonIntakeTest extends OpModePipeline {
    CRServo intake1;
    CRServo intake2;

    @Override
    public void init() {
        CRServo intake1 = hardwareMap.crservo.get("intake1");
        CRServo intake2 = hardwareMap.crservo.get("intake2");
        runMode = RobotRunMode.TELEOP;
        super.init();
    }

    public void loop() {
        Robot.update();

        intake1.setPower(gamepad1.left_stick_y);
        intake2.setPower(-gamepad1.left_stick_y);

//        subsystems.hopper.testHopperInternalServos(gamepad2.left_stick_y, gamepad2.left_stick_x);
//        subsystems.lift.run((gamepad1.right_trigger > 0) ? gamepad1.right_trigger : -gamepad1.left_trigger);
//        subsystems.lift.run(-gamepad2.left_stick_y, gamepad2.dpad_down, gamepad2.dpad_up);
//        subsystems.hopper.test(-gamepad2.left_stick_y, -gamepad2.right_stick_y);
//        subsystems.hopper.testArms(gamepad1.a, gamepad1.b);

        telemetry.update();
    }
}