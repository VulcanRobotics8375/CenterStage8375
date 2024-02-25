package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.math.control.PID;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.followers.ParametricGuidingVectorField;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.AutoPipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Toggle;
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
    PID pid = new PID(0,0,0);
    AprilTagLight pipeline;

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
        Pose2d robotPose = null;
        while (robotPose.position.y <= 40) {
            subsystems.drivetrain.mecanumDrive(1, 0, 0);
        }
        double tHeading = i*10 + 30
        while (Math.abs(subsystems.drivetrain.getIMU().getAngularOrientation().firstAngle - 30) ) {
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

}
