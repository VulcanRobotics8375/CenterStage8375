package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.Paths;
import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.robotcorelib.math.utils.MathUtils;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.followers.ParametricGuidingVectorField;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.AutoTask;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Point;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Switch;
import org.firstinspires.ftc.teamcode.vision.apriltag.testing.Projection;
import org.firstinspires.ftc.teamcode.vision.pixel.RectPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.sql.Array;
import java.util.ArrayList;
import java.util.Objects;

public class AutoTest extends OpModePipeline {
    MainConfig subsystems = new MainConfig();
    Projection pipeline = new Projection();


    double foresight;
    private OpenCvWebcam camera;

    Pose2d robotPose;

    String[][] Colors;
    int layer = 0;

    @Override
    public void init() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        super.init();
        cameraInit();
    }

    public void loop() {
        Robot.update();
//        subsystems.intake.breakBeamTelemetry();
        telemetry.addData("FPS:", camera.getFps());
        robotPose = Robot.getRobotPose();
        pipeline.updateAngles(subsystems.drivetrain.getIMU().getAngularOrientation().firstAngle / 180 * Math.PI);
        Colors = pipeline.getColors();
        telemetry.addData("X: ", robotPose.position.x);
        telemetry.addData("Y: ", robotPose.position.y);
        telemetry.update();
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

//    private Vector backboardSolver(String pixel) {
//        double mosaic = 0.0 - foresight;
//        double height = 0.0;
//        ArrayList<Point> mosaicInList = new ArrayList<>();
//        ArrayList<Point> heightInList = new ArrayList<>();
//        Vector target;
//        for (int o = 0; o < Colors.length-1; o++) {
//            for (int e = 0; e < Colors[o].length; e++) {
//                String hex = Colors[o][e];
//                Point mosaicPotential = isTouchingAdjacentMosaic(o,e);
//                if (mosaicPotential!=null) {
//                    if (Objects.equals(Colors[(int) mosaicPotential.y][(int) mosaicPotential.x], pixel)) {
//                        mosaicInList.add(mosaicPotential);
//                    }
//                }
//                height = (!Objects.equals(hex, "B"))? o / 12.0 :  height;
//            }
//        }
//
//
//    }

//    private Point isTouchingAdjacentMosaic(int o, int e) {
//        if (e == 7) {return null;}
//        if (e == 0) {return null;}
//        if (o == 12) {return null;}
//        if (o == 0) {return null;}
//        if (Colors[o][6] == null) {
//            if (Objects.equals(Colors[o][e - 1], Colors[o - 1][e])) {
//                return new Point(o,e-1);
//            }
//            else if (Objects.equals(Colors[o][e + 1], Colors[o-1][e+1])) {
//                return new Point(o,e+1);
//            }
//            else if (Objects.equals(Colors[o-1][e], Colors[o-1][e+1])) {
//                return new Point(o-1,e);
//            }
//        }
//        else {
//            if (Objects.equals(Colors[o][e - 1], Colors[o - 1][e-1])) {
//                return new Point(o,e-1);
//            }
//            else if (Objects.equals(Colors[o][e + 1], Colors[o-1][e+1])) {
//                return new Point(o,e+1);
//            }
//            else if (Objects.equals(Colors[o-1][e], Colors[o-1][e-1])) {
//                return new Point(o-1,e);
//            }
//        }
//        return null;
//    }

}
