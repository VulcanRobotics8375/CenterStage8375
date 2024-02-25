package org.firstinspires.ftc.teamcode.vision.apriltag.testing;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.checkerframework.checker.units.qual.C;
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

    final int SCREENX = 1280;
    final int SCREENY = 720;
    public double pan = 0;

    public double L = 500;
    private final long nativeApriltagPtr;

    Scalar lowerBoundY = new Scalar(0, 168, 0);
    Scalar upperBoundY = new Scalar(24, 255, 255);

    Scalar lowerBoundP = new Scalar(110, 40, 56);
    Scalar upperBoundP = new Scalar(151, 102, 255);

    Scalar lowerBoundG = new Scalar(35, 102, 0);
    Scalar upperBoundG = new Scalar(69, 255, 195);

    Scalar lowerBoundW = new Scalar(0, 0, 170);
    Scalar upperBoundW = new Scalar(63, 52, 255);

    public Projection() {
        super();
        fx = 578.272;
        fy = 578.272;
        cx = 402.145;
        cy = 221.506;

        // UNITS ARE METERS
        tagsize = 0.166;
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
        time.startTime();
    }

    static final double FEET_PER_METER = 3.28084;


    private final Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    double fx;
    double fy;
    double cx;
    double cy;

    ElapsedTime time = new ElapsedTime();

    Mat mat = new Mat();

    public double Scale = 0.6;


    // UNITS ARE METERS
    double tagsize;


    double x = 0;
    double y = 0;
    double z = 0;
    Vector3D orig = new Vector3D(0,0,0);
    Vector3D initialOrig = new Vector3D(0, 0, 1);

    public void AprilTag(Mat input) {
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);
        ArrayList<AprilTagDetection> prevD = detections;
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);
        if (detections.size() == 0) {
            detections = prevD;
        }
        synchronized (detectionsUpdateSync) {
            detectionsUpdate = detections;
        }
    }

    public Double[][][] Colors = new Double[12][7][5];
    public String[][] fullC = new String[12][7];

    ArrayList<Vector3D> hexCenters = new ArrayList<Vector3D>();

    @SuppressLint("DefaultLocale")
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        x = 0;
        y = 0;
        z = 0;
        pan = 0;
        tilt = 0;
        AprilTag(input);
        ArrayList<ArrayList<Vector3D>> polyList = new ArrayList<>();

        for (AprilTagDetection det : detections) {


            ArrayList<Vector3D> p = findTags(det.id);

            Imgproc.putText(input, String.valueOf(det.id), det.center, Imgproc.FONT_HERSHEY_COMPLEX_SMALL, 10, new Scalar(255, 255, 255), 1);
            if (p.size() == 0) {
                continue;
            }
            Orientation rot = Orientation.getOrientation(det.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.RADIANS);

            Double[] l = {Math.PI / 6, 0.0, (double) Math.PI / 2};
            p = rotate(l, p);
            AprilTagPose pose = det.pose;

            double distance = Math.sqrt(Math.pow(pose.z * FEET_PER_METER, 2) + Math.pow(pose.x * FEET_PER_METER, 2) + Math.pow(pose.y * FEET_PER_METER, 2));


            x += -p.get(0).getX() + Math.cos(rot.firstAngle) * distance;
            y += Math.sin(rot.firstAngle) * distance;
//            z += Math.sin(rot.secondAngle) * distance;
            polyList.add(p);
        }
        int size = detections.size();
        if (size > 0) {
            orig = new Vector3D(x / size, y / size, z / size).subtract(initialOrig);
        }


        for (ArrayList<Vector3D> p : polyList) {
            ArrayList<Point> normal = projection2(orig, p);
            for (int i = 0; i < normal.size(); i++) {
                Point niP = new Point(normal.get(i).x + (double) SCREENX / 2, -normal.get(i).y + (double) SCREENY / 2);
                int e = (i + 1) % normal.size();
                Point neP = new Point(normal.get(e).x + (double) SCREENX / 2, -normal.get(e).y + (double) SCREENY / 2);
                Imgproc.line(input, niP, neP, new Scalar(255, 255, 255), 5);
            }
        }
        Double[] l = {Math.PI / 8, 0.0, Math.PI / 2};
        findPixels();
//        ArrayList<Pixel> hexes = new ArrayList<>();
        int yCounter = 0, xCounter = 0;
        for (int i = 0; i < hexCenters.size(); i++) {
            Vector3D h = hexCenters.get(i);
            Double[] C = new Double[]{0.0, 0.0, 0.0, 0.0,0.0};
            ArrayList<Point> ps = projection2(orig, rotate(l, Hex(h, 0.4)));


            for (int e = 0; e < ps.size(); e++) {
                Point p = ps.get(e);
                Point p2 = ps.get((e + 1) % ps.size());
                int PX = (int) (p.x + (double) SCREENX / 2);
                int PY = (int) (-p.y + (double) SCREENY / 2);
                int PX2 = (int) (p2.x + (double) SCREENX / 2);
                int PY2 = (int) (-p2.y + (double) SCREENY / 2);
                if (PX < SCREENX && PX > 0 && PY < SCREENY && PY > 0 && PX2 < SCREENX && PX2 > 0 && PY2 < SCREENY && PY2 > 0) {
                    Scalar c = new Scalar(0, 0, 0);
                    if (inRange(mat.get(PY, PX), lowerBoundW, upperBoundW)) {
                        c = new Scalar(255, 0, 0);
                        C[0] += 1.0 / 6.0;
                    } else if (inRange(mat.get(PY, PX), lowerBoundY, upperBoundY)) {
                        c = new Scalar(255, 255, 200);
                        C[1] += 1.0 / 6.0;
                    } else if (inRange(mat.get(PY, PX), lowerBoundG, upperBoundG)) {
                        c = new Scalar(0, 255, 0);
                        C[2] += 1.0 / 6.0;
                    } else if (inRange(mat.get(PY, PX), lowerBoundP, upperBoundP)) {
                        c = new Scalar(255, 0, 255);
                        C[3] += 1.0 / 6.0;
                    }
                    else {
                        C[4] += 1.0/6.0;
                    }
                    Imgproc.line(input, new Point(PX, PY), new Point(PX2, PY2), c, 3);
                }
            }
//            hexes.add(new Pixel(new Point(AX,AY), C));
            Colors[yCounter][xCounter] = C;
            xCounter++;
            if (yCounter % 2 != 0 && xCounter == 6) {
                yCounter++;
                xCounter = 0;
            }
            if (yCounter % 2 == 0 && xCounter == 7) {
                yCounter++;
                xCounter = 0;
            }

        }
        for (int b = 0; b < Colors.length; b++) {
            for (int e = 0; e < 6; e++) {
                Double[] C = Colors[b][e];
                Point p = new Point((e + 1) * -50 + 400, (b + 1) * -50 + 500);
                int m = 0;
                for (int i = 0; i < C.length; i++) {
                    if (C[m] < C[i]) {
                        m = i;
                    }
                }
                String help = "wtf";
                if (m == 0) {
                    help = "W";
                } else if (m == 1) {
                    help = "Y";
                } else if (m == 2) {
                    help = "G";
                } else if (m == 3) {
                    help = "P";
                }
                else {
                    help = "B";
                }
                fullC[b][e] = help;

                        Imgproc.putText(input,help,p,0,1,new Scalar(255,0,0));
            }
        }

        hexCenters.clear();
        return input;
    }

    public ArrayList<Point> projection2(Vector3D orig, Iterable<Vector3D> pointList) {
        ArrayList<Point> points = new ArrayList<>();
        double cp = Math.cos(pan);
        double sp = Math.sin(pan);
        double ct = Math.cos(tilt);
        double st = Math.sin(tilt);
        for (Vector3D point : pointList) {

            double dX = point.getX() - orig.getX();
            double dY = point.getY() - orig.getY();
            double dZ = point.getZ() - orig.getZ();

            double s = ((((ct * cp * ct * cp))) + (ct * sp * ct * sp) + (st * st)) / (-(ct * cp * dX) - (ct * sp * dY) + (st * dZ));
            if (s < 0 || s > L) {
                continue;
            }
            double X = L * ((s * dX) - (ct * cp));
            double Y = L * ((ct * sp) - (s * dY));
            double Z = L * ((s * dZ) + st);

            double fX = (sp * X) + (cp * Y);
            double fY = -(((cp * X) - (sp * Y)) * st) - (Z * ct);
            points.add(new Point(fX, fY));
        }
        return points;
    }

    public ArrayList<Vector3D> rotate(Double[] t, ArrayList<Vector3D> points) {
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
                    matrix3.getEntry(0, 0),
                    matrix3.getEntry(1, 0),
                    matrix3.getEntry(2, 0)
            ));
        }
        return finalPoints;
    }

    public RealMatrix[] buildMatrix(Double[] t) {
        double cosx = Math.cos(t[0]);
        double sinx = Math.sin(t[0]);
        double cosy = Math.cos(t[1]);
        double siny = Math.sin(t[1]);
        double cosz = Math.cos(t[2]);
        double sinz = Math.sin(t[2]);
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
        double x, y, z;
        if (id == 4) {
            x = 2.5;
            y = 0;
            z = 0;
        } else if (id == 5) {
            x = 0;
            y = 0;
            z = 0;
        } else if (id == 6) {
            x = -2.5;
            y = 0;
            z = 0;
        } else {
            return new ArrayList<>();
        }
        ArrayList<Vector3D> poly = new ArrayList<>(Arrays.asList(
                new Vector3D(x, y, z),
                new Vector3D(x + Scale, y, z),
                new Vector3D(x + Scale, y, z + Scale),
                new Vector3D(x, y, z + Scale)));
        return poly;
    }

    private void findPixels() {
        for (int o = 0; o < 12; o++) {

            for (int i = 0; i < 6; i++) {
                Vector3D temp = new Vector3D(-4.5 + Scale / 2 + i * 1.65, 0, -1.2 - o * 1.2);
                if (!(o % 2 == 0)) {
                    temp = new Vector3D(-4.37 + Scale / 2 + i * 1.7 + 0.5, 0, -1.2 - o * 1.2);
                    if (i == 5) {
                        Vector3D temp2 = new Vector3D(-4.37 + Scale / 2 + (-1) * 1.7 + 0.5, 0, -1.2 - o * 1.2);
                        hexCenters.add(temp2);
                    }

                }
                hexCenters.add(temp);

            }
        }
    }

    ArrayList<Vector3D> Hex(Vector3D xyz, double scale) {
        double x = xyz.getX();
        double y = xyz.getY();
        double z = xyz.getZ();
        ArrayList<Vector3D> ret = new ArrayList<>();
        for (int i = 0; i < 6; i++) {
            ret.add(new Vector3D(x + scale * Math.cos(i * 2 * Math.PI / 6), y, z + scale * Math.sin(i * 2 * Math.PI / 6)));
        }
        return ret;

    }


    private boolean inRange(double[] pixel, Scalar lowHSV, Scalar highHSV) {
        if (pixel != null) {
            if (lowHSV.val[0] <= pixel[0] && pixel[0] <= highHSV.val[0]) {
                if (lowHSV.val[1] <= pixel[1] && pixel[1] <= highHSV.val[1]) {
                    return lowHSV.val[2] <= pixel[2] && pixel[2] <= highHSV.val[2];
                }
            }
        }

        return false;
    }

    public String[][] getColors() {return fullC;}
    public void updateAngles(double pan) {
        this.pan = pan;
    }
}
