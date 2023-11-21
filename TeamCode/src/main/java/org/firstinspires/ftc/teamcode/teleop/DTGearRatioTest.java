package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.DrivetrainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Switch;

@TeleOp(name = "DTGearRatioTest")
public class DTGearRatioTest extends OpModePipeline {
    DrivetrainConfig subsystems = new DrivetrainConfig();

    private double maxVel = 0;
    private double accelTime = 0.0;
    private ElapsedTime timer = new ElapsedTime();

    private boolean auto = false;
    private Switch aSwitch = new Switch();

    @Override
    public void init() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        super.init();
    }

    public void loop() {
        Robot.update();

        Pose2d robotVel = Robot.getRobotVelocity();

        if(Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_x) > 0.01) {
            auto = false;
            subsystems.drivetrain.mechanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            if(aSwitch.simpleSwitch(gamepad1.a)) {
                auto = !auto;
                if(auto) {
                    timer.reset();
                }
            }
            if(auto && Math.abs(robotVel.getX()) > maxVel) {
                maxVel = Math.abs(robotVel.getX());
                accelTime = timer.milliseconds();
            }
            subsystems.drivetrain.mechanumDrive(auto ? 1 : 0, 0, 0);
        }

        telemetry.addData("vel", robotVel.toString());
        telemetry.addData("max vel", maxVel);
        telemetry.addData("accelTime", accelTime);

        telemetry.update();
    }
}
