package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.math.utils.MathUtils;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.followers.ParametricGuidingVectorField;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.AutoPipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.AutoTask;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Toggle;
import org.firstinspires.ftc.teamcode.vision.pixel.RectPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Auto")
public class Auto extends AutoPipeline {
    private OpenCvWebcam camera;
    private RectPipeline pipeline = new RectPipeline();
    private double speed = 0.65;

    private Toggle redToggle = new Toggle(), backDropToggle = new Toggle(), parkToggle = new Toggle();
    private boolean red = true, backDrop = true, parkLeft = true;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        ParametricGuidingVectorField follower = new ParametricGuidingVectorField(this);

        Paths paths = new Paths(speed);

        MainConfig subsystems = new MainConfig();

        //------------------------------------------------------------------------------------------------------------------------------

        super.subsystems = subsystems;
        runMode = RobotRunMode.AUTONOMOUS;
        robotInit();

        subsystems.lift.liftPID.setSampleTime(60);
        subsystems.lift.liftPID.setP(0.01);
        subsystems.lift.liftPID.setI(0.0004);
        subsystems.hopper.doorClose();
        subsystems.hopper.hopperDown();

        subsystems.intake.armUp();
        cameraInit();

        while (!isStarted() && !isStopRequested())
        {
            if (redToggle.toggle(gamepad1.a)) {
                red = !red;
                pipeline.detectRed(red);
            }
            backDrop = backDropToggle.toggle(gamepad1.b) ? !backDrop : backDrop;
            parkLeft = parkToggle.toggle(gamepad1.x) ? !parkLeft : parkLeft;

            telemetry.addData("Red side", red);
            telemetry.addData("Backdrop side", backDrop);
            telemetry.addData("Left park", parkLeft);

            telemetry.addData("first idx", ((red && backDrop) || !(red || backDrop)) ? 0 : 1);
            telemetry.addData("second idx", pipeline.getPropIdx());

            telemetry.update();
            sleep(20);
        }

        //------------------------------------------------------------------------------------------------------------------------------

        Robot.setRobotPose(new Pose2d());

        subsystems.droneLauncher.launcherUp();
        subsystems.intake.armDown();
        timer.reset();
        runTask(new AutoTask() {
            @Override
            public boolean conditional() {
                return timer.milliseconds() <= 1500;
            }

            @Override
            public void run() {
                telemetry.addData("prop Idx", pipeline.getPropIdx());
                telemetry.update();
            }
        });
        camera.stopStreaming();
        int propIdx = pipeline.getPropIdx();
        subsystems.droneLauncher.launcherDown();

        follower.followPath(paths.getSpikeMarkPath(red, backDrop, propIdx));

        subsystems.intake.armUp();
        subsystems.intake.counterRoller.setPower(-0.7);
        timer.reset();
        runTask(new AutoTask() {
            @Override
            public boolean conditional() {
                return timer.milliseconds() <= 1500;
            }

            @Override
            public void run() {
            }
        });

        Pose2d robotPose = Robot.getRobotPose();
        robotPose = new Pose2d(robotPose.getX(), robotPose.getY(), MathUtils.angleWrap(robotPose.getHeading()));
        follower.followPathAsync(paths.getBackDropPath(robotPose, red, backDrop, propIdx));
        boolean hardStop = false;
        while(!follower.atEnd && !hardStop && !isStopRequested()) {
            follower.update();
            if(follower.distanceFromEnd < 40) {
                subsystems.lift.runToFirstPixel();
                subsystems.hopper.hopperUp();
                if (Robot.getRobotPose().vec().distTo(new Vector2d(follower.xSpline.value(1.0), follower.ySpline.value(1.0))) < 1.0 && Robot.getRobotVelocity().vec().norm() < 0.2) {
                    hardStop = true;
                }
            }
        }

        subsystems.hopper.doorOpen();
        timer.reset();
        runTask(new AutoTask() {
            @Override
            public boolean conditional() {
                return timer.milliseconds() <= 3000;
            }

            @Override
            public void run() {
            }
        });

        follower.followPathAsync(paths.getParkPath(red, parkLeft, propIdx));
        timer.reset();
        while (!follower.atEnd && !isStopRequested()) {
            follower.update();
            if(timer.milliseconds() > 1000) {
                subsystems.hopper.hopperDown();
                subsystems.lift.home();
                subsystems.hopper.doorClose();
            }
        }

        Robot.stop();
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
