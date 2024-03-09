package org.firstinspires.ftc.teamcode.vision.apriltag;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.AprilTagPose;

import java.util.ArrayList;

public class AprilTagLight extends AprilTagDetectionPipeline {
    long nativeApriltagPtr;
    public AprilTagLight() {
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

    Scalar lowBlueBound = new Scalar(100, 40, 40); // set lower and upper bounds for the color we want to recognize (blue in this case)
    Scalar highBlueBound = new Scalar(125, 500, 500); //hue is out of 180 here, double it if searching it up online due to online websites going from 0-360
    private final Mat grey = new Mat();
    public ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    double fx;
    double fy;
    double cx;
    double cy;
    Rect leftRect = new Rect(190, 270, 200, 100); // define our regions of interest (where the algorithm is focusing on) as rectangles
    Rect midRect = new Rect(570, 260, 530, 100);
    Rect rightRect = new Rect(800, 270, 200, 100);

    ElapsedTime time = new ElapsedTime();

    Mat mat = new Mat();
    Mat hsvColor = new Mat();

    public double pan = 0.0;
    double x = 0.0, y = 0.0;
    final double PERCENT_THRESHOLD = 0.19; // define our threshold


    public boolean red = false;



    // UNITS ARE METERS
    double tagsize;

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
    public void updateAngles(double pan) {
        this.pan = pan;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public Mat processFrame(Mat input) {
        mat = input;
        AprilTag(mat);
        return mat;
    }

    private double AprilTagYValues(int id) {
        if (red) {
            switch (id) {
                case 1:
                    return 0.0;
                case 2:
                    return 0.0;
                case 3:
                    return 0.0;
                case 4:
                    return 0.0;
                case 5:
                    return 0.0;
                case 6:
                    return 0.0;
            }
        }
        else {
            switch (id) {
                case 1:
                    return 0.0;
                case 2:
                    return 0.0;
                case 3:
                    return 0.0;
                case 4:
                    return 0.0;
                case 5:
                    return 0.0;
                case 6:
                    return 0.0;
            }
        }
        return 0.0;
    }

    public Pose2d getPose() {
        double x = 0, y = 0;
        for (AprilTagDetection det : detections) {
            AprilTagPose pose = det.pose;

            double distance = Math.sqrt(Math.pow(pose.z * FEET_PER_METER, 2) + Math.pow(pose.x * FEET_PER_METER, 2) + Math.pow(pose.y * FEET_PER_METER, 2));

            y += -AprilTagYValues(det.id) + Math.cos(pan) * distance;
            x += Math.sin(pan) * distance;
        }
        x /= detections.size();
        y /= detections.size();
        return new Pose2d(x,y,0);
    }
    public int getProp(Scalar lowBound, Scalar highBound) {
        Imgproc.cvtColor(mat, hsvColor, Imgproc.COLOR_RGB2HSV); // change the color space from rgb to HSV (Hue, Saturation, Value)
        Core.inRange(hsvColor, lowBound, highBound, hsvColor); // see which pixels are in our range, convert the pixels we're looking for into white, store it back to mat i think
        Mat left = mat.submat(leftRect); // create sub-matrices for our regions of interest
        Mat middle = mat.submat(midRect);
        Mat right = mat.submat(rightRect);

        double leftValue = Core.sumElems(left).val[0] / leftRect.area() / 255; // get the percentage of white pixels that are present
        double midValue = Core.sumElems(middle).val[0] / midRect.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / rightRect.area() / 255;
        Imgproc.rectangle(mat, leftRect, new Scalar(255,255,255), 2); // draw the rectangles on the output matrix
//        Imgproc.rectangle(mat, rightRect, blue, 2);
        Imgproc.rectangle(mat, midRect, new Scalar(255,255,255), 2);
        Imgproc.rectangle(mat, rightRect, new Scalar(255,255,255), 2);

        if (leftValue > midValue && leftValue > rightValue) {
            return 1;
        }
        else if (midValue > leftValue && midValue > rightValue) {
            return 2;
        }
        else if (rightValue > leftValue && rightValue > midValue) {
            return 3;
        }
        else {
            return 0;
        }
    }

}