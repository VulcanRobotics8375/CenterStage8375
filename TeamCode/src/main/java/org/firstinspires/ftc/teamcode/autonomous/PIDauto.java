package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.math.control.PID;
import org.firstinspires.ftc.teamcode.robotcorelib.math.utils.MathUtils;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.followers.ParametricGuidingVectorField;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.followers.PurePursuit;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.Path;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.PathBuilder;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.AutoPipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Toggle;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.vision.apriltag.AprilTagLight;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class PIDauto extends AutoPipeline {
    OpenCvWebcam camera;
    ElapsedTime timer = new ElapsedTime();
    private Toggle redToggle = new Toggle(), backDropToggle = new Toggle(), parkToggle = new Toggle();
    boolean red;
    PID pidX = new PID(0,0,0);
    PID pidT = new PID(0,0,0);
    PID pidY = new PID(0,0,0);
    AprilTagLight pipeline;
    double S=0.65;
    double ALLOWED_POSE_ERROR = 1;
    double ALLOWED_HEADING_ERROR = 3;

    Pose2d robotPose;
    Pose2d robotVelocity;
    double poseError;
    double headingError;

    double speed;
    double turnSpeed;

    final double POSE_ERROR_GAIN = 1.00001;
    final double HEADING_ERROR_GAIN = 1.0001;



    PurePursuit purePursuit = new PurePursuit();

    public void runOpMode() {
        cameraInit();
        MainConfig subsystems = new MainConfig();
        super.subsystems = subsystems;
        runMode = RobotRunMode.AUTONOMOUS;
        robotInit();
        while (!isStarted() && !isStopRequested()) {
            if (redToggle.toggle(gamepad1.a)) {
                pipeline.red = !pipeline.red;
            }
        }

        follow(new Pose2d(0,0,0), subsystems.drivetrain);
        if (pipeline.detections.size()>=3) {
            Robot.setRobotPose(pipeline.getPose());
        }



    }

    private void follow(Pose2d pose, Drivetrain dt) {
        while((poseError > ALLOWED_POSE_ERROR || headingError > Math.toRadians(ALLOWED_HEADING_ERROR))) {
            Robot.update();
            robotPose = Robot.getRobotPose();
            robotVelocity = Robot.getRobotVelocity();
            poseError = Math.hypot(pose.getX() - robotPose.getX(), pose.getY() -  robotPose.getY());
            headingError = MathUtils.calcAngularError(pose.getHeading(), robotPose.getHeading());
            speed = poseError * POSE_ERROR_GAIN; //pose gain
            turnSpeed = headingError * HEADING_ERROR_GAIN; //heading gain
            double idealX = Math.sin(pose.getHeading())*speed;
            double idealY = Math.cos(pose.getHeading())*speed;

            dt.mecanumDrive(pidX.getOutput(robotVelocity.getX(), idealX), pidY.getOutput(robotVelocity.getY(), idealY),pidT.getOutput(robotPose.getHeading(),pose.getHeading())); // y = cos, x = sin
        }
    }

    private void cameraInit() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "front_camera"), cameraMonitorViewId);
        camera.setPipeline(pipeline);
        camera.showFpsMeterOnViewport(false);
        camera.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }
}