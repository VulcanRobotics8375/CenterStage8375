//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.autonomous.Paths;
//import org.firstinspires.ftc.teamcode.robot.MainConfig;
//import org.firstinspires.ftc.teamcode.robotcorelib.motion.followers.ParametricGuidingVectorField;
//import org.firstinspires.ftc.teamcode.robotcorelib.opmode.AutoPipeline;
//import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
//import org.firstinspires.ftc.teamcode.robotcorelib.util.Toggle;
//import org.firstinspires.ftc.teamcode.vision.apriltag.testing.Projection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//public class AutoDepoTest extends AutoPipeline {
//    private OpenCvWebcam camera;
//
//    Projection pipeline;
//    private double speed = 0.65;
//
//    private Toggle redToggle = new Toggle(), backDropToggle = new Toggle(), parkToggle = new Toggle();
//    private boolean red = true, backDrop = true, parkLeft = true;
//
//    ElapsedTime timer = new ElapsedTime();
//
//    @Override
//    public void runOpMode() {
//        ParametricGuidingVectorField follower = new ParametricGuidingVectorField(this);
//
//        Paths paths = new Paths(speed);
//
//        MainConfig subsystems = new MainConfig();
//
//        //------------------------------------------------------------------------------------------------------------------------------
//
//        super.subsystems = subsystems;
//        runMode = RobotRunMode.AUTONOMOUS;
//        robotInit();
//        cameraInit();
//        while (!isStarted() && !isStopRequested())
//        {
//            if (redToggle.toggle(gamepad1.a)) {
//                red = !red;
//                pipeline.detectRed(red);
//            }
//            backDrop = backDropToggle.toggle(gamepad1.b) ? !backDrop : backDrop;
//            parkLeft = parkToggle.toggle(gamepad1.x) ? !parkLeft : parkLeft;
//
//            telemetry.addData("Red side", red);
//            telemetry.addData("Backdrop side", backDrop);
//            telemetry.addData("Left park", parkLeft);
//
//            telemetry.addData("first idx", ((red && backDrop) || !(red || backDrop)) ? 0 : 1);
//            telemetry.addData("second idx", pipeline.getPropIdx());
//
//            telemetry.update();
//            sleep(20);
//        }
//    }
//    private void cameraInit() {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "front_camera"), cameraMonitorViewId);
//        camera.setPipeline(pipeline);
//        camera.showFpsMeterOnViewport(false);
//        camera.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//            }
//        });
//    }
//}
