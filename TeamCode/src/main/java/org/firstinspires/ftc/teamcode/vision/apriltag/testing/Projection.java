package org.firstinspires.ftc.teamcode.vision.apriltag.testing;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.AprilTagPose;

import java.util.ArrayList;
import java.util.Arrays;

public class Projection extends AprilTagDetectionPipeline {

    public double tilt = 0;

    final int SCREENX = 640; final int SCREENY = 1200;
    public double pan= 0;

    public double L = 150;
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

    static final double FEET_PER_METER = 3.28084;


    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    double fx;
    double fy;
    double cx;
    double cy;

    public double Scale = 10;


    double pi = 3.141;


    // UNITS ARE METERS
    double tagsize;


    double x = 0; double y = 0; double z = 0;


    Vector3D orig;
    public double Scaler = 10.0;

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
        x = 0; y = 0; z = 0;
        pan = 0; tilt= 0;
        AprilTag(input);
        ArrayList<ArrayList<Vector3D>> polyList = new ArrayList<>();

        for (AprilTagDetection det : detections) {
            ArrayList<Vector3D> p = findTags(det.id);
            Imgproc.putText(input, String.valueOf(det.id), det.center, Imgproc.FONT_HERSHEY_COMPLEX_SMALL, 10, new Scalar(255,255,255),1);
            if (p.size()==0){continue;}
            Double[] l = {0.25, 0.0, 1.5};
            p = rotate(l, p);
            AprilTagPose pose = det.pose;
            x += p.get(0).getX() + pose.z * FEET_PER_METER;
            y += p.get(0).getY() + pose.x * FEET_PER_METER;
            z += p.get(0).getZ() + pose.y * FEET_PER_METER;
            Orientation rot = Orientation.getOrientation(det.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.RADIANS);

            pan += rot.thirdAngle;
            tilt += rot.firstAngle;
            polyList.add(p);
        }
        int size = polyList.size();
        Imgproc.putText(input, String.valueOf(size), new Point(100,100), Imgproc.FONT_HERSHEY_COMPLEX_SMALL, 10, new Scalar(255,255,255),1);
        orig = new Vector3D(x/size,y/size,z/size);
        pan /= size;
        tilt /= size;
        for (ArrayList<Vector3D> p : polyList) {
            ArrayList<Point> normal = projection2(orig, p);
            for (int i=0; i < normal.size(); i++) {

                Point niP = new Point(normal.get(i).x + (double) SCREENX / 2, -normal.get(i).y + (double) SCREENY / 2);
                int e = (i + 1) % normal.size();
                Point neP = new Point(normal.get(e).x + (double) SCREENX / 2, -normal.get(e).y + (double) SCREENY / 2);
                Imgproc.line(input, niP, neP, new Scalar(255, 255, 255), 5);
            }
        }
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



    private void translate(ArrayList<Vector3D> v, Vector3D translate){
        for (Vector3D v3 : v) {
            double vX = v3.getX() + translate.getX();
            double vY = v3.getY() + translate.getY();
            double vZ = v3.getZ() + translate.getZ();
            new Vector3D(vX, vY, vZ);
        }
    }
    private ArrayList<Vector3D> TempTranslate(ArrayList<Vector3D> v, Vector3D translate){
        ArrayList<Vector3D> vector3DS=new ArrayList<>();
        for (Vector3D v3 : v) {
            double vX = v3.getX() + translate.getX();
            double vY = v3.getY() + translate.getY();
            double vZ = v3.getZ() + translate.getZ();
            vector3DS.add(new Vector3D(vX, vY, vZ));
        }
        return vector3DS;
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

    private ArrayList<Vector3D> findTags(int id) {
        double x,y,z;
//        if (id == 1){x = 0; y = 0; z = 0;}
//        else if (id == 2){x = 0; y = 0; z = 0;}
//        else if (id == 3){x = 0; y = 0; z = 0;}
        if (id == 4){x = 0; y = 0; z = 0;}
        else if (id == 5){x = 6; y = 0; z = 0;}
        else if (id == 6){x = 6; y = 0; z = 0;}
        else {return new ArrayList<>();}
        return new ArrayList<>(Arrays.asList(
                new Vector3D(x,y,z),
                new Vector3D(x+Scale,y,z),
                new Vector3D(x+Scale,y,z+Scale),
                new Vector3D(x,y,z+Scale)));
    }
}
