package org.firstinspires.ftc.teamcode.vision.props;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teamcode.Pipelines.EOCV.RedPipelineForLeft;
import org.firstinspires.ftc.teamcode.teamcode.Teleop;
import org.firstinspires.ftc.teamcode.teamcode.VoyagerBot;
import org.firstinspires.ftc.teamcode.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Red Left with PIDF for the arm", group = "Red")
public class RedLeftWithPIDF extends LinearOpMode {
    private PIDController controller;
    public static double p = 0.009, i = 0.0, d = 0.0007;
    public static double f = 0.2;

    public static double target = 10;
    int delay = 0;
    private final double ticks_in_degree = 1425.1 / 360;

    //    public static double elbowPos = 0.01;
    //           VoyagerBot robot = new VoyagerBot();
    //       robot.init(hardwareMap);

    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        VoyagerBot robot = new VoyagerBot();
        robot.init(hardwareMap);

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        RedPipelineForLeft pipeline;
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

        pipeline = new RedPipelineForLeft(telemetry);
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

        RedPipelineForLeft.Location location = null;
        while (!isStarted()) {
            location = pipeline.getLocation();
            telemetry.addData("Location", location);
            telemetry.update();
        }
        TrajectorySequence test = null;

        waitForStart();

        // Set starting position
        Pose2d startPose = new Pose2d(-35, -63, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        if (location == RedPipelineForLeft.Location.LEFT) {
            delay = 10;
            test = drive.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90)))

                    .addTemporalMarker( 0.1 + delay, () -> {
                        robot.elbow.setPosition(0.05);
                    })
                    .addTemporalMarker( 0.2 + delay, () -> {
                        robot.elbow.setPosition(0.1);
                    })
                    .addTemporalMarker( 0.3 + delay, () -> {
                        robot.elbow.setPosition(0.15);
                    })
                    .addTemporalMarker( 0.4 + delay, () -> {
                        robot.elbow.setPosition(0.2);
                    })
                    .addTemporalMarker(0.5 + delay, () -> {
                        robot.elbow.setPosition(0.27);
                    })
                    .addTemporalMarker(1.5 + delay, () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftOpenPos);
                    })
                    .addTemporalMarker(2 + delay,  () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftClosePos);
                        robot.elbow.setPosition(0.21);
                        target = 140;
                    })
                    .addTemporalMarker(9 + delay, () -> {
                        robot.clawRight.setPosition(Teleop.clawRightOpenPos);
                    })
//                    .addTemporalMarker(7, () -> {
//                        robot.clawRight.setPosition(Teleop.clawRightClosePos);
//                        //  robot.runShoulderTo(40, 0.1);
//                    })

                    .waitSeconds(delay)
                    .lineToLinearHeading(new Pose2d(-32, -32, Math.toRadians(180)))
                    //TODO: Add temporal marker for the elbow to go to best height
                    .forward(0.1)
                    .waitSeconds(0.5)
                    .back(1)
                    .lineToLinearHeading(new Pose2d(-34.5, -12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(34.6, -12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(41.3, -23.9, Math.toRadians(0)))
                    .forward(13)//to backboard
                    .waitSeconds(1)
                    .lineToLinearHeading(new Pose2d(44.9, -23.9, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(44.9, -55, Math.toRadians(0)))
                    .forward(3)

                    .build();

            //TODO: cycle code here
            // test = drive.trajectorySequenceBuilder(new Pose2d(-35, 63, Math.toRadians(-90)))
//                    .lineToLinearHeading(new Pose2d(-32, 35, Math.toRadians(0)))

//            .lineToLinearHeading(new Pose2d(-50, 38, Math.toRadians(180)))
//                    .waitSeconds(1)
//                    .forward(
//                            3,
//                            SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .addTemporalMarker(7.3,  () -> {
//                        robot.clawLeft.setPosition(Teleop.clawLeftClosePos);
//                    })
//
//             .waitSeconds(1)
//                    .lineToLinearHeading(new Pose2d(-48, 37, Math.toRadians(180))) // goes back
//                    .waitSeconds(1)
//                    .lineToLinearHeading(new Pose2d(-48, 11.3, Math.toRadians(180)))
//                    .lineToLinearHeading(new Pose2d(34,11.3, Math.toRadians(180)))

//             .lineToLinearHeading(new Pose2d(50, 41, Math.toRadians(180)))
//                    .waitSeconds(1)
////                    .back(
////                            2,
////                            SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
////                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
////                    )
//                    .waitSeconds(2)




        } else if (location == RedPipelineForLeft.Location.MIDDLE) {
            delay = 6;
            test = drive.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
//            drive.trajectorySequenceBuilder(new Pose2d(-35, 63, Math.toRadians(-90)))
//                    .lineToLinearHeading(new Pose2d(-35.1, 34.4, Math.toRadians(-90)))
//                    .waitSeconds(1.5)
//                    .lineToLinearHeading(new Pose2d(-35.1, 38, Math.toRadians(-90)))
//                    .lineToLinearHeading(new Pose2d(-50, 38, Math.toRadians(180))) 5 and 8, second to last
//                    .build()
                    .addTemporalMarker( 0.1 + delay, () -> {
                        robot.elbow.setPosition(0.05);
                    })
                    .addTemporalMarker( 0.2 + delay, () -> {
                        robot.elbow.setPosition(0.1);
                    })
                    .addTemporalMarker( 0.3 + delay, () -> {
                        robot.elbow.setPosition(0.15);
                    })
                    .addTemporalMarker( 0.4 + delay, () -> {
                        robot.elbow.setPosition(0.2);
                    })
                    .addTemporalMarker(0.5 + delay, () -> {
                        robot.elbow.setPosition(0.27);
                    })
                    .addTemporalMarker(2 + delay, () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftOpenPos);
                    })
                    .addTemporalMarker(2.2 + delay, () -> {
                        robot.elbow.setPosition(0.15);
                    })
                    .addTemporalMarker(2.1 + delay,  () -> {
                        target = 150;
                    })
                    .addTemporalMarker(5 + delay, () -> {
                        robot.elbow.setPosition(0.2);
                    })
                    .addTemporalMarker(5.1 + delay, () -> {
                        robot.elbow.setPosition(0.15);
                    })
                    .addTemporalMarker(6.9 + delay, () -> {
                        target = 74;
                    })
                    .addTemporalMarker(8 + delay,  () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftClosePos);
                    })
                    .addTemporalMarker(8.5 + delay,  () -> {
                        robot.elbow.setPosition(0.1);
                    })
                    .addTemporalMarker(12.4 + delay,  () -> {
                        target = 120;
                        robot.elbow.setPosition(0);
                    })
                    .addTemporalMarker(12.5 + delay,  () -> {
                        target = 200;
                    })
                    .addTemporalMarker(13 + delay,  () -> {
                        target = 500;
                    })
                    .addTemporalMarker(14 + delay,  () -> {
                        target = 600;
                    })
                    .addTemporalMarker(14.5 + delay,  () -> {
                        target = 650;
                    })
                    .addTemporalMarker(14.6 + delay,  () -> {
                        target = 710; // runs to backdrop position
                    })
                    .addTemporalMarker(16.5 + delay, () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftClosePos);
                        robot.clawRight.setPosition(Teleop.clawRightOpenPos);
                    })
                    .addTemporalMarker(17 + delay, () -> {
                        target = 500;
                        robot.clawLeft.setPosition(Teleop.clawLeftClosePos);
                        robot.clawRight.setPosition(Teleop.clawRightClosePos);
                    })
                    .addTemporalMarker(19.5 + delay, () -> {
                        target = 400;
                    })
                    .addTemporalMarker(20 + delay, () -> {
                        target = 300;
                    })
                    .addTemporalMarker(20.5 + delay, () -> {
                        target = 200;
                    })
                    .addTemporalMarker(21 + delay, () -> {
                        target = 100;
                    })
                    .addTemporalMarker(21.5 + delay, () -> {
                        target = 25;
                    })

                    .waitSeconds(delay)
                    .lineToLinearHeading(new Pose2d(-30, -36, Math.toRadians(90)))
                    .waitSeconds(1.5)
                    .lineToLinearHeading(new Pose2d(-50, -39, Math.toRadians(177)))
                    .waitSeconds(3)
                    .forward(3)
                    .waitSeconds(1)
                    .lineToLinearHeading(new Pose2d(-45, -11.3, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(34,-11.3, Math.toRadians(180)))

                    .lineToLinearHeading(new Pose2d(50, -34.6, Math.toRadians(180)))
                    .waitSeconds(3)
                    .forward(6)

                    .strafeRight(-23)
                    .back(10)
                    .waitSeconds(10)
                    .build();

        } else if (location == RedPipelineForLeft.Location.RIGHT){
            delay = 9;
            test = drive.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
                    .addTemporalMarker( 0.1 + delay, () -> {
                        robot.elbow.setPosition(0.05);
                    })
                    .addTemporalMarker( 0.2 + delay, () -> {
                        robot.elbow.setPosition(0.1);
                    })
                    .addTemporalMarker( 0.3 + delay, () -> {
                        robot.elbow.setPosition(0.15);
                    })
                    .addTemporalMarker( 0.4 + delay, () -> {
                        robot.elbow.setPosition(0.2);
                    })
                    .addTemporalMarker(0.5 + delay, () -> {
                        robot.elbow.setPosition(0.27);
                    })
                    .addTemporalMarker(2 + delay, () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftOpenPos);
                    })
                    .addTemporalMarker(2.05 + delay, () -> {
                        target = 150;
                        robot.elbow.setPosition(0.27);
                    })
                    .addTemporalMarker(4.5 + delay, () -> {
                        target = 140;
                    })
                    .addTemporalMarker(6 + delay, () -> {
                        robot.clawLeft.setPosition(Teleop.clawLeftClosePos);
                    })
                    .addTemporalMarker(7 + delay, () -> {
                        robot.elbow.setPosition(0.23);
                        target = 100;
                    })
                    .addTemporalMarker(13.5 + delay, () -> {
                        target = 200;
                    })
                    .addTemporalMarker(17.7 + delay, () -> {
                        robot.clawRight.setPosition(Teleop.clawRightOpenPos);
                    })

                    //TODO: Add temporal marker for the claw to drop the pixel
                    //TODO: Add temporal marker for the shoulder and elbow and claw to go to the correct height
                    // target: 125, maybe do 120, and elbow is 0.38
                    .addTemporalMarker(20 + delay, () -> {
                        target = 20;
                        robot.clawRight.setPosition(Teleop.clawRightClosePos);
                        robot.clawLeft.setPosition(Teleop.clawLeftClosePos);
                    })
                    .waitSeconds(delay)
                    .lineToLinearHeading(new Pose2d(-35, -35, Math.toRadians(0)))
                    .waitSeconds(2)
                    // TODO: target: 125, maybe do 120, and elbow is 0.38
                    .lineToLinearHeading(new Pose2d(-50, -35, Math.toRadians(180)))
                    .waitSeconds(1)
                    .forward(2,
                            SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                            )
                    .waitSeconds(1)
                    .lineToLinearHeading(new Pose2d(-48, -37, Math.toRadians(180))) // goes back
                    .waitSeconds(1)
                    .lineToLinearHeading(new Pose2d(-33, -11.3, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(34,-11.3, Math.toRadians(0))) // goes across gate
                    .lineToLinearHeading(new Pose2d(50, -42.1, Math.toRadians(0)))
                    .waitSeconds(2)
                    .forward(5)
                    .back(8)

                    .strafeLeft(-19)
                    //.back(4)
                    .waitSeconds(10)

                    /* code without stack
                    .lineToLinearHeading(new Pose2d(-36, -35, Math.toRadians(0)))
                                .waitSeconds(2)
                                // TODO: target: 125, maybe do 120, and elbow is 0.38

                                .lineToLinearHeading(new Pose2d(-33, -11.3, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(34,-11.3, Math.toRadians(0))) // goes across gate
                                .lineToLinearHeading(new Pose2d(50, -41, Math.toRadians(0)))
                                .waitSeconds(2)
                                .forward(6)
                                .back(8)

                                .strafeLeft(18)
                                //.back(4)
                                .waitSeconds(10)

                     */



                    .build();


        } else { //the left trajectory is going to be our failsafe
            //TODO: Update this with the best preforming one
            test = drive.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90)))

                    .addTemporalMarker( 0.1, () -> {
                        robot.elbow.setPosition(0.05);
                    })
                    .addTemporalMarker( 0.2, () -> {
                        robot.elbow.setPosition(0.1);
                    })
                    .addTemporalMarker( 0.3, () -> {
                        robot.elbow.setPosition(0.15);
                    })
                    .addTemporalMarker( 0.4, () -> {
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
                        robot.runShoulderTo(100, 0.3);;
                    })
                    .addTemporalMarker(9, () -> {
                        robot.clawRight.setPosition(Teleop.clawRightOpenPos);
                    })
//                    .addTemporalMarker(7, () -> {
//                        robot.clawRight.setPosition(Teleop.clawRightClosePos);
//                        //  robot.runShoulderTo(40, 0.1);
//                    })

                    .lineToLinearHeading(new Pose2d(-32, -32, Math.toRadians(180)))
                    //TODO: Add temporal marker for the elbow to go to best height
                    .forward(0.1)
                    .waitSeconds(0.5)
                    .back(1)
                    .lineToLinearHeading(new Pose2d(-34.5, -12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(34.6, -12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(41.3, -23.9, Math.toRadians(0)))
                    .forward(13)//to backboard
                    .waitSeconds(1)
                    .lineToLinearHeading(new Pose2d(44.9, -23.9, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(44.9, -10.3, Math.toRadians(0)))
                    .forward(3)

                    .build();

            //TODO: cycle code here
            // test = drive.trajectorySequenceBuilder(new Pose2d(-35, 63, Math.toRadians(-90)))
//                    .lineToLinearHeading(new Pose2d(-32, 35, Math.toRadians(0)))

//            .lineToLinearHeading(new Pose2d(-50, 38, Math.toRadians(180)))
//                    .waitSeconds(1)
//                    .forward(
//                            3,
//                            SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .addTemporalMarker(7.3,  () -> {
//                        robot.clawLeft.setPosition(Teleop.clawLeftClosePos);
//                    })
//
//             .waitSeconds(1)
//                    .lineToLinearHeading(new Pose2d(-48, 37, Math.toRadians(180))) // goes back
//                    .waitSeconds(1)
//                    .lineToLinearHeading(new Pose2d(-48, 11.3, Math.toRadians(180)))
//                    .lineToLinearHeading(new Pose2d(34,11.3, Math.toRadians(180)))

//             .lineToLinearHeading(new Pose2d(50, 41, Math.toRadians(180)))
//                    .waitSeconds(1)
////                    .back(
////                            2,
////                            SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
////                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
////                    )
//                    .waitSeconds(2)




        }

        waitForStart();

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
