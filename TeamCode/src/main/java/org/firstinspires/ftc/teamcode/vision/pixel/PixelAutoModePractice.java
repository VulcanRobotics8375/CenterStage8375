package org.firstinspires.ftc.teamcode.vision.pixel;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.openftc.easyopencv.*;

@Autonomous(name="Pixel Detector", group="Auto")

public class PixelAutoModePractice extends LinearOpMode {
    OpenCvCamera phoneCam;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                    "id", hardwareMap.appContext.getPackageName());

        phoneCam = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        PixelDetectorPractice detector = new PixelDetectorPractice(telemetry);
        phoneCam.setPipeline(detector);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();
        switch(detector.getLocation()) {
            case LEFT:
                break;
            case RIGHT:
                break;
            case NOT_FOUND:
                //...
        }
        phoneCam.stopStreaming();
    }


}
