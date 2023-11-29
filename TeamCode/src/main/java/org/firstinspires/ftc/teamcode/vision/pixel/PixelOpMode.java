package org.firstinspires.ftc.teamcode.vision.pixel;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
public class PixelOpMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private final int rows = 720;
    private final int cols = 1280;

    OpenCvWebcam webcam;

    public static int Lx = 422;
    public static int Ly = 383;
    public static int La = 17;
    public static int Lc = 73;
    public static int Ld = 58;
    public static int Cx = 176;
    public static int Cy = 383;
    public static int Ca = 190;
    public static int Cc = 6;
    public static int Cd = 0;
    public static int Rx = 103;
    public static int Ry = 383;
    public static int Ra = 17;
    public static int Rc = 73;
    public static int Rd = -76;

    @Override
    public void runOpMode() throws InterruptedException {
        PixelCounterPipeline pipeline = new PixelCounterPipeline();

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "front_camera"), cameraMonitorViewId);

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        webcam.setPipeline(pipeline);

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(cols, rows, OpenCvCameraRotation.UPSIDE_DOWN);
                //width, height
                //width = height in this case, because camera is in portrait mode.
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
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
