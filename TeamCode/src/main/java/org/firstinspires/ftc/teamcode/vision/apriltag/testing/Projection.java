package org.firstinspires.ftc.teamcode.vision.apriltag.testing;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.AprilTagPose;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;

public class Projection extends AprilTagDetectionPipeline {

    public double tilt = 0;

    final int SCREENX = 640; final int SCREENY = 800;
    public double pan= 0;

    public double L = 150;

    public double origx, origy,origz=0;

    private long nativeApriltagPtr;

    public Projection() {
        super();
        fx = 578.272;
        fy = 578.272;
        cx = 402.145;
        cy = 221.506;

        // UNITS ARE METERS
        tagsize = 0.166;
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);

    }

    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    double fx;
    double fy;
    double cx;
    double cy;

    // UNITS ARE METERS
    double tagsize;


    public void AprilTag(Mat input) {
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);
        synchronized (detectionsUpdateSync)
        {
            detectionsUpdate = detections;
        }
    }
    @Override
    public Mat processFrame(Mat input) {
        AprilTag(input);

        Vector3D orig = new Vector3D(origx,origy,origz);
        for (AprilTagDetection det : detections) {
            AprilTagPose pose = det.pose;
            double x = pose.x;
            double y = pose.y;
            double z = pose.z;
            double Scale = 10;
            ArrayList<Vector3D> p = new ArrayList<>(Arrays.asList(new Vector3D(x,y,z),
                    new Vector3D(x,y-Scale,z),
                    new Vector3D(x-Scale,y-Scale,z),
                    new Vector3D(x-Scale,y,z),
                    new Vector3D(x,y,z),
                    new Vector3D(x,y,z-Scale),
                    new Vector3D(x-Scale,y,z-Scale),
                    new Vector3D(x-Scale,y-Scale,z-Scale),
                    new Vector3D(x,y-Scale,z-Scale),
                    new Vector3D(x,y,z-Scale)));
            pan = det.pose.R.get(0,0);
            ArrayList<Point> normal = projection2(orig, p);
//            Double[] l = {(double) pose.R.get(0, 0), (double) pose.R.get(0, 1), (double) pose.R.get(0, 2)};
            Double[] l = {0.0,30.0,0.0};
            p = rotate(l, p);
            for (int i=0; i < normal.size(); i++) {

                Point niP = new Point(normal.get(i).x + SCREENX/2, -normal.get(i).y + SCREENY/2);
                int e = (i + 1) % normal.size();
                Point neP = new Point(normal.get(e).x + SCREENX/2, -normal.get(e).y + SCREENY/2);
                Imgproc.line(input, niP, neP, new Scalar(255, 255, 255), 15);
            }
        }
//
//
//        ArrayList<Vector3D> ps = new ArrayList<>(Arrays.asList(
//                new Vector3D(10,10,10),
//                new Vector3D(10,-10,10),
//                new Vector3D(-10,-10,10),
//                new Vector3D(-10,10,10),
//                new Vector3D(10,10,10),
//                new Vector3D(10,10,-10),
//                new Vector3D(-10,10,-10),
//                new Vector3D(-10,-10,-10),
//                new Vector3D(10,-10,-10),
//                new Vector3D(10,10,-10)));
//        Double[] list = {0.0,30.0,0.0};
//        ps = rotate(list, ps);
//        Imgproc.line(input,new Point(orig.getX(), orig.getZ()),new Point(orig.getX(), orig.getZ()),new Scalar(0,0,0), 50);
//
//        ArrayList<Point> normalP = projection2(orig, ps);
//
//        for (int i=0; i < normalP.size(); i++){
//
//            Point niP = new Point(normalP.get(i).x + 900, -normalP.get(i).y + 1200);
//            int e = (i + 1) % normalP.size();
//            Point neP = new Point(normalP.get(e).x + 900, -normalP.get(e).y + 1200);
//            Imgproc.line(input, niP, neP, new Scalar(100, 100, 255), 15);
//
////            Imgproc.line(input, niP, new Point(900,1200), new Scalar(100, 100, 255), 15);
//
//
//            Imgproc.putText(input, Integer.toString(i), niP, Imgproc.FONT_HERSHEY_DUPLEX, 5, new Scalar(255,0,0));
//
//        }
        return input;
    }

    public ArrayList<Point> projection2(Vector3D orig, Iterable<Vector3D> pointList) {
        ArrayList<Point> points = new ArrayList<>();
        double cp = Math.cos(pan)+0.00001;
        double sp = Math.sin(pan)+0.00001 ;
        double ct = Math.cos(tilt)+0.00001;
        double st = Math.sin(tilt)+0.00001;
        for (Vector3D point : pointList) {

            double dX = point.getX() - orig.getX();
            double dY = point.getY() - orig.getY();
            double dZ = point.getZ() - orig.getZ();

            double s = ((((ct*cp*ct*cp)))+(ct*sp*ct*sp)+(st*st))/(-(ct*cp*dX)-(ct*sp*dY)+(st*dZ));
            if (s < 0 || s>L){continue ;}
            double X = L*((s*dX)-(ct*cp));
            double Y = L*((ct*sp)-(s*dY));
            double Z = L*((s*dZ)+st);        

            double fX = (sp*X)+(cp*Y);
            double fY = (((cp*X)-(sp*Y))*st)-(Z*ct);
            points.add(new Point(fX, fY));
        }

        return points;
    }

    public ArrayList<Vector3D> rotate(Double[] t, ArrayList<Vector3D> points){
        RealMatrix[] RMs = buildMatrix(t);
        ArrayList<Vector3D> finalPoints = new ArrayList<>();
        for (Vector3D point3 : points) {
            RealMatrix matrix3 = MatrixUtils.createRealMatrix(new double[][]{
                    {point3.getX()},
                    {point3.getY()},
                    {point3.getZ()}
            });
            for (RealMatrix Rm : RMs) {
                matrix3 = Rm.multiply(matrix3);
            }
            finalPoints.add(new Vector3D(
                    matrix3.getEntry(0,0),
                    matrix3.getEntry(1,0),
                    matrix3.getEntry(2,0)
            ));
        }
        return finalPoints;
    }

    public RealMatrix[] buildMatrix(Double[] t) {
        double cosx = Math.cos(t[0]); double sinx = Math.sin(t[0]);
        double cosy = Math.cos(t[1]); double siny = Math.sin(t[1]);
        double cosz = Math.cos(t[2]); double sinz = Math.sin(t[2]);
        RealMatrix[] matrixList = new RealMatrix[3];
        matrixList[0] = MatrixUtils.createRealMatrix(new double[][]{
                {1, 0, 0},
                {0, cosx, -sinx},
                {0, sinx, cosx}
        });

        matrixList[1] = MatrixUtils.createRealMatrix(new double[][]{
                {cosy, 0, siny},
                {0, 1, 0},
                {-siny, 0, cosy}
        });

        matrixList[2] = MatrixUtils.createRealMatrix(new double[][]{
                {cosz, -sinz, 0},
                {sinz, cosz, 0},
                {0, 0, 1}
        });
        return matrixList;
    }
}
