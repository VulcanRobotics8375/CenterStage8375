package org.firstinspires.ftc.teamcode.vision.apriltag.testing;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.AprilTagPose;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class AprilTagDetectionPipeline extends OpenCvPipeline {
    private long nativeApriltagPtr;
    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    Mat cameraMatrix;

    Scalar blue = new Scalar(7, 197, 235, 255);
    Scalar red = new Scalar(255, 0, 0, 255);
    Scalar green = new Scalar(0, 255, 0, 255);
    Scalar white = new Scalar(255, 255, 255, 255);

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    public int move = 0;

    // UNITS ARE METERS
    double tagsize = 0.166;
    double tagsizeX = 0.166;
    double tagsizeY = 0.166;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    public AprilTagDetectionPipeline() {


        constructMatrix();

        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    public void finalize() {
        // Might be null if createApriltagDetector() threw an exception
        if (nativeApriltagPtr != 0) {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        } else {
            System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
        }
    }

    private ArrayList<Point3> rotate3dZ(double t, ArrayList<Point3> plist) {
        double cos = Math.cos(t);
        double sin = Math.sin(t);
        ArrayList<Point3> reP = new ArrayList<>();
        for (Point3 p : plist) {
            double x = p.x * cos - p.y * sin;
            double y = p.y * cos + p.x * sin;
            reP.add(new Point3(x,y,p.z));
        }
        return reP;
    }
    private ArrayList<Point3> rotate3dX(double t, Point3 axis, ArrayList<Point3> plist) {
        double cos = Math.cos(t);
        double sin = Math.sin(t);
        ArrayList<Point3> reP = new ArrayList<>();
        for (Point3 p : plist) {
            double y = p.y * cos - p.z * sin;
            double z = p.z * cos + p.y * sin;
            reP.add(new Point3(p.x,y,z));
        }
        return reP;
    }
    private ArrayList<Point3> rotate3dY(double t, ArrayList<Point3> plist) {
        double cos = Math.cos(t);
        double sin = Math.sin(t);
        ArrayList<Point3> reP = new ArrayList<>();
        for (Point3 p : plist) {
            double x = p.x * cos + p.z * sin;
            double z = p.z * cos - p.x * sin;
            reP.add(new Point3(x,p.y,z));
        }
        return reP;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Convert to greyscale
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

        synchronized (decimationSync) {
            if (needToSetDecimation) {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                needToSetDecimation = false;
            }
        }

        // Run AprilTag
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

        synchronized (detectionsUpdateSync) {
            detectionsUpdate = detections;
        }

        // For fun, use OpenCV to draw 6DOF markers on the image.
        double S1 = 0;
        double S2 = 0;
        double centerX = 0;
        double B2 = 0;
        Point3 pos = new Point3(0,0,0);
        for (AprilTagDetection det : detections) {
            S1 += (det.corners[0].y - det.corners[3].y) / (det.corners[0].x - det.corners[3].x);
            S2 += (det.corners[3].y - det.corners[2].y) / (det.corners[3].x - det.corners[2].x);
            pos.x += det.pose.x;
            pos.y += det.pose.y;
            pos.z += det.pose.z;

            centerX += det.center.x;

            B2 -= ((S2 * det.corners[3].x) - det.corners[3].y);
        }

        int avgSize = detections.size();

        double finalS = S1/avgSize;
        double finalS2 = S2/avgSize;


        double finalCenterX = centerX/avgSize;

        pos.x = pos.x/avgSize;
        pos.y = pos.y/avgSize;
        pos.z = pos.z/avgSize;

        double finalB2  = B2/avgSize;




        double scale = 1 / dist3d(pos);
        double xset = 10000 * scale;
        double xScale = 2000 * scale;
        double yScale = 5 * scale;

        Point kys = new Point(finalCenterX - xScale,  (S2 * (finalCenterX - xScale))+B2);
        Point kys2 = new Point(finalCenterX + xScale, (S2 * (finalCenterX + xScale))+B2);
        double B1 = -((S1*kys.x) - kys.y);
        double B12 = -((S1*kys2.x) - kys2.y);

        ArrayList<Point> dtList = new ArrayList<Point>() {
            {
                add(new Point(finalCenterX - xScale,  (finalS2 * (finalCenterX - xScale))+ finalB2));
                add(new Point(finalCenterX + xScale, (finalS2 * (finalCenterX + xScale))+ finalB2));
                add(new Point((finalCenterX + yScale)-xset, (finalS * ((finalCenterX + yScale)-xset))+B12));
                add(new Point((finalCenterX - yScale)-xset, (finalS * ((finalCenterX - yScale)-xset))+B1));

            }

        };

        for (int e=0; e < 4; e++) {
            if (e>0) {
                Imgproc.line(input, dtList.get(e), dtList.get(e - 1), new Scalar(255,0,0),25);
            }
            else{
                Imgproc.line(input, dtList.get(e), dtList.get(3),new Scalar(255,0,0),25);
            }
        }

//            dtList = rotate3dX(aX, dtList);
//            dtList = rotate3dY(aY, dtList);
//            dtList = rotate3dZ(aZ, dtList);
//
//            for (Point3 pont : dtList) {
//                Imgproc.line(input, new Point(pont.x, pont.y), new Point(pont.x, pont.y), new Scalar(255, 255, 255), 100);
//            }

        return input;
    }

    public double dist3d(Point3 pos1) {
        return Math.sqrt((Math.pow(pos1.x, 2) + Math.pow(pos1.y, 2) + Math.pow(pos1.z, 2)));
    }

    public void setDecimation(float decimation) {
        synchronized (decimationSync) {
            this.decimation = decimation;
            needToSetDecimation = true;
        }
    }

    public ArrayList<AprilTagDetection> getLatestDetections() {
        return detections;
    }

    public ArrayList<AprilTagDetection> getDetectionsUpdate() {
        synchronized (detectionsUpdateSync) {
            ArrayList<AprilTagDetection> ret = detectionsUpdate;
            detectionsUpdate = null;
            return ret;
        }
    }

    void constructMatrix() {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);

        cameraMatrix.put(0, 0, fx);
        cameraMatrix.put(0, 1, 0);
        cameraMatrix.put(0, 2, cx);

        cameraMatrix.put(1, 0, 0);
        cameraMatrix.put(1, 1, fy);
        cameraMatrix.put(1, 2, cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2, 1, 0);
        cameraMatrix.put(2, 2, 1);
    }

    /**
     * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
     *
     * @param buf          the RGB buffer on which to draw the marker
     * @param length       the length of each of the marker 'poles'
     * @param rvec         the rotation vector of the detection
     * @param tvec         the translation vector of the detection
     * @param cameraMatrix the camera matrix used when finding the detection
     */
    void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix) {
        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(0, 0, 0),
                new Point3(length, 0, 0),
                new Point3(0, length, 0),
                new Point3(0, 0, -length)
        );

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Draw the marker!
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness);

        Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
    }

    void draw3dCubeMarker(Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix) {
        //axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
        //       [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(-tagWidth / 2, tagHeight / 2, 0),
                new Point3(tagWidth / 2, tagHeight / 2, 0),
                new Point3(tagWidth / 2, -tagHeight / 2, 0),
                new Point3(-tagWidth / 2, -tagHeight / 2, 0),
                new Point3(-tagWidth / 2, tagHeight / 2, -length),
                new Point3(tagWidth / 2, tagHeight / 2, -length),
                new Point3(tagWidth / 2, -tagHeight / 2, -length),
                new Point3(-tagWidth / 2, -tagHeight / 2, -length));

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Pillars
        for (int i = 0; i < 4; i++) {
            Imgproc.line(buf, projectedPoints[i], projectedPoints[i + 4], blue, thickness);
        }

        // Base lines
        //Imgproc.line(buf, projectedPoints[0], projectedPoints[1], blue, thickness);
        //Imgproc.line(buf, projectedPoints[1], projectedPoints[2], blue, thickness);
        //Imgproc.line(buf, projectedPoints[2], projectedPoints[3], blue, thickness);
        //Imgproc.line(buf, projectedPoints[3], projectedPoints[0], blue, thickness);

        // Top lines
        Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness);
        Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness);
        Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness);
        Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness);
    }

    Pose aprilTagPoseToOpenCvPose(AprilTagPose aprilTagPose) {
        Pose pose = new Pose();
        pose.tvec.put(0, 0, aprilTagPose.x);
        pose.tvec.put(1, 0, aprilTagPose.y);
        pose.tvec.put(2, 0, aprilTagPose.z);

        Mat R = new Mat(3, 3, CvType.CV_32F);

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R.put(i, j, aprilTagPose.R.get(i, j));
            }
        }

        Calib3d.Rodrigues(R, pose.rvec);

        return pose;
    }

    /**
     * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
     * original size of the tag.
     *
     * @param points       the points which form the trapezoid
     * @param cameraMatrix the camera intrinsics matrix
     * @param tagsizeX     the original width of the tag
     * @param tagsizeY     the original height of the tag
     * @return the 6DOF pose of the camera relative to the tag
     */
    Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsizeX, double tagsizeY) {
        // The actual 2d points of the tag detected in the image
        MatOfPoint2f points2d = new MatOfPoint2f(points);

        // The 3d points of the tag in an 'ideal projection'
        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-tagsizeX / 2, tagsizeY / 2, 0);
        arrayPoints3d[1] = new Point3(tagsizeX / 2, tagsizeY / 2, 0);
        arrayPoints3d[2] = new Point3(tagsizeX / 2, -tagsizeY / 2, 0);
        arrayPoints3d[3] = new Point3(-tagsizeX / 2, -tagsizeY / 2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

        // Using this information, actually solve for pose
        Pose pose = new Pose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

        return pose;
    }

    /*
     * A simple container to hold both rotation and translation
     * vectors, which together form a 6DOF pose.
     */
    class Pose {
        Mat rvec;
        Mat tvec;

        public Pose() {
            rvec = new Mat(3, 1, CvType.CV_32F);
            tvec = new Mat(3, 1, CvType.CV_32F);
        }

        public Pose(Mat rvec, Mat tvec) {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }
}
