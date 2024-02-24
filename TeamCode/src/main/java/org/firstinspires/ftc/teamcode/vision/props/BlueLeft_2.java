package org.firstinspires.ftc.teamcode.vision.props;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teamcode.Pipelines.EOCV.BluePipeline;
import org.firstinspires.ftc.teamcode.teamcode.Teleop;
import org.firstinspires.ftc.teamcode.teamcode.VoyagerBot;
import org.firstinspires.ftc.teamcode.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Blue Left", group = "Blue")
public class BlueLeft extends LinearOpMode {

    private PIDController controller;
    public static double p = 0.01, i = 0.0, d = 0.0007;
    public static double f = 0.2;

    public static int target = 5;
    private final double ticks_in_degree = 1425.1 / 360;
    //    public static double elbowPos = 0.01;
    //           VoyagerBot robot = new VoyagerBot();
    //       robot.init(hardwareMap);

    public static double pShoulder = 0.01, iShoulder = 0.0, dShoulder = 0.001;

    public static int targetShoulder;

    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        VoyagerBot robot = new VoyagerBot();
        robot.init(hardwareMap);
//        robot.elbow.setPosition(0.05);
        robot.clawLeft.setPosition(Teleop.clawLeftClosePos);
        robot.clawRight.setPosition(Teleop.clawRightClosePos);

        controller = new PIDController(p, i, d);

        BluePipeline pipeline;
        OpenCvWebcam webcam;
        String webcamName = "webcamFront";

        //add the positions EVERY SINGLE MECHANISM SHOULD BE IN
        //DO robot.resetEncoders() in here as well

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
                );

        webcam = OpenCvCameraFactory
                .getInstance().createWebcam(
                        hardwareMap.get(WebcamName.class, webcamName),
                        cameraMonitorViewId);

        pipeline = new BluePipeline(telemetry);
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        BluePipeline.Location location = null;
        while (!isStarted()) {
            location = pipeline.getLocation();
            telemetry.addData("Location", location);
            telemetry.update();
        }
        TrajectorySequence test = null;

        // PID setup for shoulder

//        int armPos = robot.shoulder.getCurrentPosition();
//
//        controller.setPID(pShoulder, iShoulder, dShoulder);
//
//        double pid = controller.calculate(armPos, targetShoulder);
//        double ff = Math.cos(Math.toRadians(targetShoulder / ticks_in_degree)) * f;
//
//        double power = pid + ff;


        waitForStart();

        // Set starting position
        Pose2d startPose = new Pose2d(12, 63, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        if (location == BluePipeline.Location.LEFT) {
            test = drive.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(34.3, 35, Math.toRadians(180)))
                    .addTemporalMarker(0, () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftClosePos);
                        robot.clawRight.setPosition(Teleop.clawRightClosePos);
                    })
                    .addTemporalMarker( 0, () -> {
                        robot.elbow.setPosition(0.1);
                    })
                    .addTemporalMarker( 0.3, () -> {
                        robot.elbow.setPosition(0.2);
                    })
                    .addTemporalMarker(0.5, () -> {
                        robot.elbow.setPosition(0.27);
                    })
                    .addTemporalMarker(1.5, () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftOpenPos);
                    })
                    .addTemporalMarker(2,  () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftClosePos);
                        robot.elbow.setPosition(0.21);
                        target = 100;
                    })

                    .waitSeconds(0.5)

                    .lineToLinearHeading(new Pose2d(45, 45, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(56, 45, Math.toRadians(0)))
                    .waitSeconds(1)
                    .addTemporalMarker(5.6,  () -> {
                        robot.clawRight.setPosition(Teleop.clawRightOpenPos);
                    })
                    .addTemporalMarker(7.6, () -> {
                        robot.clawRight.setPosition(Teleop.clawRightClosePos);
                    })
                    .waitSeconds(1)

                    .back(10)
                    .addTemporalMarker(7.6, () -> {
                        target = 40;
                    })
                    .addTemporalMarker(8.6, () -> {
                        robot.elbow.setPosition(0);
                    })
                    .splineToConstantHeading(new Vector2d(54, 12), Math.toRadians(0))
                    .waitSeconds(1)
                    .build();

        } else if (location == BluePipeline.Location.MIDDLE) {
            targetShoulder = 120;
            test = drive.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(17, 37, Math.toRadians(-90)))
                    .addTemporalMarker(0, () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftClosePos);
                        robot.clawRight.setPosition(Teleop.clawRightClosePos);
                    }).addTemporalMarker(0, () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftClosePos);
                        robot.clawRight.setPosition(Teleop.clawRightClosePos);
                    })
                    .addTemporalMarker( 0, () -> {
                        robot.elbow.setPosition(0.1);
                    })
                    .addTemporalMarker( 0.3, () -> {
                        robot.elbow.setPosition(0.2);
                    })
                    .addTemporalMarker(0.5, () -> {
                        robot.elbow.setPosition(0.27);
                    })
                    .waitSeconds(0.5)
                    .addTemporalMarker(1.4, () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftOpenPos);

                    })
                    .addTemporalMarker(1.65, () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftClosePos);
                        robot.elbow.setPosition(0.21);
                        target = targetShoulder;

                    })
                    .back(3.5)

                    .lineToLinearHeading(new Pose2d(45, 39, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(56, 38.5, Math.toRadians(0)))
                    .addTemporalMarker(4.9, () -> {
                        robot.clawRight.setPosition(Teleop.clawRightOpenPos);

                    })
                    .addTemporalMarker(5.2, () -> {
                        robot.clawRight.setPosition(Teleop.clawRightClosePos);
                        robot.elbow.setPosition(0.1);
                        //maybe add arm stuff here

                    })

                    .waitSeconds(0.5)
                    .back(10)

                    .strafeRight(12)

                    .splineToConstantHeading(new Vector2d(55, 12), Math.toRadians(0))
                    .addDisplacementMarker( () -> {
                        target = 10;
                    })
                    .build();
        } else if (location == BluePipeline.Location.RIGHT){
            test = drive.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(-90)))
                    .addTemporalMarker(0, () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftClosePos);
                        robot.clawRight.setPosition(Teleop.clawRightClosePos);
                    }).addTemporalMarker(0, () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftClosePos);
                        robot.clawRight.setPosition(Teleop.clawRightClosePos);
                    })
                    .addTemporalMarker( 0, () -> {
                        robot.elbow.setPosition(0.1);
                    })
                    .addTemporalMarker( 0.3, () -> {
                        robot.elbow.setPosition(0.2);
                    })
                    .addTemporalMarker(0.5, () -> {
                        robot.elbow.setPosition(0.27);
                    })
                    .lineToLinearHeading(new Pose2d(14, 36, Math.toRadians(-180)))

                    .forward(0.6)
                    .addTemporalMarker(1.82, () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftOpenPos);
                    })
                    .addTemporalMarker(1.9, () -> {
                        robot.elbow.setPosition(0.21);
                    })
                    .addTemporalMarker(2.6, () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftClosePos);
                        robot.elbow.setPosition(0.17);
                        target = 140;
                    })
                    .waitSeconds(0.5)
                    .back(0.6)

                    .lineToLinearHeading(new Pose2d(47, 31.5, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(56, 31.5, Math.toRadians(0)))

                    .addTemporalMarker(5.4, () -> {
                        robot.clawRight.setPosition(Teleop.clawRightOpenPos);
                    })
                    .waitSeconds(1)

                    .back(11)
                    .addTemporalMarker(8, () -> {
                        robot.clawRight.setPosition(Teleop.clawRightClosePos);
                        target = 40;
                        robot.elbow.setPosition(0.1);
                    })
                    .strafeRight(4)

                    .splineToConstantHeading(new Vector2d(56, 13), Math.toRadians(0))
                    .waitSeconds(0.5)
                    .build();

        } else { //the left trajectory is going to be our failsafe

            test = drive.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(34.3, 35, Math.toRadians(180)))
                    .addTemporalMarker(0, () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftClosePos);
                        robot.clawRight.setPosition(Teleop.clawRightClosePos);
                    })
                    .addTemporalMarker( 0, () -> {
                        robot.elbow.setPosition(0.1);
                    })
                    .addTemporalMarker( 0.3, () -> {
                        robot.elbow.setPosition(0.2);
                    })
                    .addTemporalMarker(0.5, () -> {
                        robot.elbow.setPosition(0.27);
                    })
                    .addTemporalMarker(1.5, () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftOpenPos);
                    })
                    .addTemporalMarker(2,  () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftClosePos);
                        robot.elbow.setPosition(0.21);
                        target = 100;
                    })

                    .waitSeconds(0.5)

                    .lineToLinearHeading(new Pose2d(45, 45, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(56, 45, Math.toRadians(0)))
                    .waitSeconds(1)
                    .addTemporalMarker(5.6,  () -> {
                        robot.clawRight.setPosition(Teleop.clawRightOpenPos);
                    })
                    .addTemporalMarker(7.6, () -> {
                        robot.clawRight.setPosition(Teleop.clawRightClosePos);
                    })
                    .waitSeconds(1)

                    .back(10)
                    .addTemporalMarker(7.6, () -> {
                        target = 40;
                    })
                    .addTemporalMarker(8.6, () -> {
                        robot.elbow.setPosition(0);
                    })
                    .splineToConstantHeading(new Vector2d(54, 12), Math.toRadians(0))
                    .waitSeconds(1)
                    .build();

        }
        drive.followTrajectorySequenceAsync(test);
        telemetry.addData("shoulder position", robot.shoulder.getCurrentPosition());



        while(opModeIsActive()){
            drive.update();
            int armPos = robot.shoulder.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double power = pid + ff;
            robot.shoulder.setPower(power);
        }


//        target = 650;
//        controller.setPID(p, i, d);
//        int armPos = robot.shoulder.getCurrentPosition();
//
//        double pid = controller.calculate(armPos, target);
//        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
//
//        double power = pid + ff;

//        robot.shoulder.setPower(power); does not work at all, may need to do async path following

    }
}
