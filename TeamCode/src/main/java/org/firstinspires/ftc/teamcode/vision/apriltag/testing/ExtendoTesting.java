package org.firstinspires.ftc.teamcode.vision.apriltag.testing;

import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ExtendoTesting extends AprilTagDetectionPipeline {


    Scalar whiteLow = new Scalar(0, 0, 170);
    Scalar whiteHigh = new Scalar(63, 52, 255);

    Mat mask = new Mat();
    Mat mat = new Mat();

    ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    Mat hierarchy = new Mat();

    double pan = 0.0;

    ArrayList<Rect> rects = new ArrayList<>();

    static final double FEET_PER_METER = 3.28084;


    private final Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    double fx;
    double fy;
    double cx;
    double cy;
    private long nativeApriltagPtr;
    final int SCREENX = 1280;
    final int SCREENY = 720;
    double tagsize;

    public ExtendoTesting() {
        super();
        fx = 578.272;
        fy = 578.272;
        cx = 402.145;
        cy = 221.506;

        // UNITS ARE METERS
        tagsize = 0.166;
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

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

    @Override
    public Mat processFrame(Mat input) {
        if (detections.size() != 1) {return input;}
        AprilTagDetection det = detections.get(0);

    }
}
