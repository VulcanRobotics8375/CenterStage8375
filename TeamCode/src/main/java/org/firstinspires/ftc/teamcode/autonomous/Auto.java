package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.followers.ParametricGuidingVectorField;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.Path;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.PathBuilder;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.AutoPipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@Autonomous(name="Auto")
public class Auto extends AutoPipeline {
    @Override
    public void runOpMode() {
        ParametricGuidingVectorField follower = new ParametricGuidingVectorField(this);
        Path coolPath = new PathBuilder()
                .speed(0.5)
                .turnSpeed(0.5)
                .maintainHeading(true)
                .start(new Pose2d(0, 0, 0))
                .addGuidePoint(new Pose2d(10,10,0))
                .end(new Pose2d(20, 10, 0))
                .build();
        MainConfig subsystems = new MainConfig();


        super.subsystems = subsystems;
        runMode = RobotRunMode.AUTONOMOUS;
        robotInit();

        while (!isStarted() && !isStopRequested())
        {
            telemetry.update();
            sleep(20);
        }

//        subsystems.drivetrain.setPowers(0.15, 0.15, 0.15, 0.15);
//        sleep(3000);
        Robot.setRobotPose(new Pose2d());
        follower.followPath(coolPath);

        Robot.stop();
    }
}
