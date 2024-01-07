package org.firstinspires.ftc.teamcode.vision.apriltag;

import org.opencv.core.Mat;
import org.opencv.core.Point3;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class AprilTagPose extends OpenCvPipeline {

    Mat mat = new Mat();
    ArrayList<AprilTagDetection> dtList = new ArrayList<AprilTagDetection>();
    ArrayList<AprilTagDetection> ringBuffer = new ArrayList<AprilTagDetection>();
    public void update(ArrayList<AprilTagDetection> dtList){
        this.dtList = dtList;

    }
    public void FiFoUpdate(AprilTagDetection dt) { ringBuffer.add(dt); }

    public AprilTagDetection accessBuf() {
//        if (dtList.size() == 0) { return null; }
        AprilTagDetection dt =  ringBuffer.get(0);
        ringBuffer.remove(0);
        return dt;
    }

    public Mat getBackBoardMat (Mat input){
        boolean scannable = (dtList.size() == 3);
        for (AprilTagDetection det : dtList) {
            scannable = (scannable & (det.id == 6 || det.id == 5 || det.id == 4));
        }
        if (scannable) {
            double angley=0;
            double anglex=0;
            for (AprilTagDetection det : dtList) {
                angley += det.pose.y;
                anglex += det.pose.x;
            }
            anglex = anglex/3;
            angley = angley/3;
        }
        return input;
    }

    private void rotate3dZ(double t, ArrayList<Point3> plist) {
        double cos = Math.cos(t);
        double sin = Math.sin(t);
        for (Point3 p : plist) {
            double x = p.x * cos - p.y * sin;
            double y = p.y * cos + p.x * sin;
            p = new Point3(x,y,p.z);
        }
    }
    private void rotate3dX(double t, ArrayList<Point3> plist) {
        double cos = Math.cos(t);
        double sin = Math.sin(t);
        for (Point3 p : plist) {
            double y = p.y * cos - p.z * sin;
            double z = p.z * cos + p.y * sin;
            p = new Point3(p.x,y,z);
        }
    }
    private void rotate3dY(double t, ArrayList<Point3> plist) {
        double cos = Math.cos(t);
        double sin = Math.sin(t);
        for (Point3 p : plist) {
            double x = p.x * cos + p.z * sin;
            double z = p.z * cos - p.x * sin;
            p = new Point3(x,p.y,z);
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        mat = input;
        return mat;
    }
}
