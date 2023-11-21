package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.firstinspires.ftc.vision.*;
import org.firstinspires.ftc.vision.apriltag.*;

import org.openftc.easyopencv.*;


@TeleOp
public class Vision extends LinearOpMode {
    OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                        .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();


        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            if (tagProcessor.getDetections().size() > 1) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.x);
                telemetry.addData("z", tag.ftcPose.x);
                telemetry.addData("roll", tag.ftcPose.x);
                telemetry.addData("pitch", tag.ftcPose.x);
                telemetry.addData("yaw", tag.ftcPose.x);
            }

            telemetry.update();
        }
    }


}
