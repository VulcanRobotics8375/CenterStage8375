package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.math.control.SimplePID;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

import java.util.Arrays;

@TeleOp(name = "ExampleOpMode")
public class ExampleOpMode extends OpModePipeline {
    MainConfig subsystems = new MainConfig();
    double secTime = -1;
    double deltaTime = 0;
    Pose2d vel = new Pose2d();
    Pose2d position = new Pose2d();
    SimplePID turnPID = new SimplePID(0.04, 0.0, 0.0, -1.0, 1.0);
    double turnPower = 0.0;

    @Override
    public void init() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        super.init();
    }

    public void loop() {
        Robot.update();

        subsystems.intake.run(gamepad1.a);
        subsystems.flywheel.run(gamepad1.x, gamepad1.y, gamepad1.b);

        Acceleration accel = subsystems.drivetrain.getIMU().getAcceleration();

        if (secTime > 0) {
            deltaTime = accel.acquisitionTime/1000000000.0 - secTime;
            Pose2d deltaVel = new Pose2d(accel.xAccel, accel.yAccel).times(deltaTime);
            Pose2d deltaPosition = vel.plus(deltaVel.div(2.0));
            vel = vel.plus(deltaVel);
            position = position.plus(deltaPosition);
        }
        secTime = accel.acquisitionTime;

//        double angle = subsystems.drivetrain.getIMU().getAngularOrientation().firstAngle/180*Math.PI;
//        turnPower = -turnPID.run(0, angle);
//        subsystems.drivetrain.setPowers(DriveKinematics.mecanumFieldVelocityToWheelVelocities(new Pose2d(0, 0, angle), new Pose2d(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)));
        subsystems.drivetrain.mechanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


//        telemetry.addData("robot angle", angle);
        telemetry.addData("robot pose", Arrays.toString(new double[] {position.getX(), position.getY()}));
        telemetry.addData("robot acceleration", Arrays.toString(new double[] {accel.xAccel, accel.yAccel}));

        telemetry.update();
    }
}
