package org.firstinspires.ftc.teamcode.vision.pixel;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.AutoPipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
import org.firstinspires.ftc.teamcode.vision.pixel.PixelCounterPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name= "PixelOpMode")
public class PixelOpMode extends AutoPipeline {
    private ElapsedTime runtime = new ElapsedTime();

    private final int rows = 720;
    private final int cols = 1280;

    OpenCvWebcam webcam;

    public static int Rx = 402;
    public static int Ry = 400;
    public static int Ra = 16;
    public static int Rc = 76;
    public static int Rd = 48;
    public static int Cx = 176;
    public static int Cy = 383;
    public static int Ca = 190;
    public static int Cc = 6;
    public static int Cd = 0;
    public static int Lx = 103;
    public static int Ly = 383;
    public static int La = 17;
    public static int Lc = 73;
    public static int Ld = -76;

    @Override
    public void runOpMode() throws InterruptedException {
        PixelCounterPipeline pipeline = new PixelCounterPipeline();
        MainConfig subsystems = new MainConfig();

        super.subsystems = subsystems;
        runMode = RobotRunMode.AUTONOMOUS;
        robotInit();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "front_camera"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.showFpsMeterOnViewport(false);
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(cols, rows, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (!isStarted() && !isStopRequested())
        {
            pipeline.Lx = Lx;
            pipeline.Ly = Ly;
            pipeline.La = La;
            pipeline.Lc = Lc;
            pipeline.Ld = Ld;
            pipeline.Cx = Cx;
            pipeline.Cy = Cy;
            pipeline.Ca = Ca;
            pipeline.Cc = Cc;
            pipeline.Cd = Cd;
            pipeline.Rx = Rx;
            pipeline.Ry = Ry;
            pipeline.Ra = Ra;
            pipeline.Rc = Rc;
            pipeline.Rd = Rd;
            
//            subsystems.droneLauncher.launcherUp();
            telemetry.addData("count", pipeline.getFinalcount());

            telemetry.update();
            sleep(20);
        }

        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);



            telemetry.update();
            sleep(100);
            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);
        }
    }
}
