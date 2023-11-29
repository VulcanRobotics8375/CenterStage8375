package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@TeleOp
public class OdometryTest extends OpModePipeline {

    MainConfig subsystems = new MainConfig();

    public void init() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        super.init();
    }

    public void start() {
        super.start();

    }


    @Override
    public void loop() {
        Robot.update();

        Pose2d robotPose = Robot.getRobotPose();
        telemetry.addData("left wheel", subsystems.localizer.getWheelPositions().get(0));
        telemetry.addData("right wheel", subsystems.localizer.getWheelPositions().get(1));
        telemetry.addData("center wheel", subsystems.localizer.getWheelPositions().get(2));

        telemetry.addData("robot pose", robotPose.getX() + ", " + robotPose.getY() + ", " + robotPose.getHeading());

        telemetry.update();
    }
}
