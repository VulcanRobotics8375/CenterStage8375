package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.followers.ParametricGuidingVectorField;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.Path;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.PathBuilder;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.AutoPipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
import org.firstinspires.ftc.teamcode.vision.pixel.PixelCounterPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Auto")
public class Auto extends AutoPipeline {
    OpenCvCamera camera;
    @Override
    public void runOpMode() {
        PixelCounterPipeline pixelCounterPipeline = new PixelCounterPipeline();
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
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.setPipeline(pixelCounterPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera
                camera.startStreaming(1280, 800, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
                Robot.addErrorMessage("camera no work");
            }
        });


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
