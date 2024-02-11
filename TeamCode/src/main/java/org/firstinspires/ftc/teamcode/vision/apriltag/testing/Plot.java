package org.firstinspires.ftc.teamcode.vision.apriltag.testing;




import com.qualcomm.robotcore.util.ElapsedTime;

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

import java.util.ArrayList;

public class Plot extends AprilTagDetectionPipeline {
    Mat mat;
    ElapsedTime time=new ElapsedTime();
    ArrayList<Point> log = new ArrayList<>();
    ArrayList<Point> log1 = new ArrayList<>();
    ArrayList<Point> log2 = new ArrayList<>();


    ArrayList<Point> log4 = new ArrayList<>();
    ArrayList<Point> log5 = new ArrayList<>();
    ArrayList<Point> log6 = new ArrayList<>();
    ArrayList<Point> Klog;
    private long nativeApriltagPtr;

    public Plot() {
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


    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    double fx;
    double fy;
    double cx;
    double cy;
    double tagsize;

    public boolean reset = false;

    public void AprilTag(Mat input) {
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);
        synchronized (detectionsUpdateSync) {
            detectionsUpdate = detections;
        }
    }
    @Override
    public Mat processFrame(Mat input) {
        mat = input;
        AprilTag(input);



//        for (AprilTagDetection det : detections) {
//            Orientation rot = Orientation.getOrientation(det.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.RADIANS);
//            log.add(new Point(time.milliseconds() * 0.05, rot.firstAngle*200+200));
//            log1.add(new Point(time.milliseconds() * 0.05, rot.secondAngle*200+200));
//            log2.add(new Point(time.milliseconds() * 0.05, rot.thirdAngle*200+200));
//
//
//        }

        if (detections.size()>0){
            Orientation rot = Orientation.getOrientation(detections.get(0).pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.RADIANS);
            Point p1 = new Point(time.milliseconds() * 0.05, rot.firstAngle*200+200);
            Point p2 = new Point(time.milliseconds() * 0.05, rot.secondAngle*200+200);
            Point p3 = new Point(time.milliseconds() * 0.05, rot.thirdAngle*200+200);
            Point p4 = new Point(time.milliseconds() * 0.05, detections.get(0).pose.x*100+200);
            Point p5 = new Point(time.milliseconds() * 0.05, detections.get(0).pose.y*100+200);
            Point p6 = new Point(time.milliseconds() * 0.05, detections.get(0).pose.z*100+200);
            Imgproc.putText(mat, Double.toString(rot.firstAngle), p1, 5, Imgproc.FONT_HERSHEY_COMPLEX_SMALL, new Scalar(255,255,255));
            Imgproc.putText(mat, Double.toString(rot.secondAngle), p2, 5, Imgproc.FONT_HERSHEY_COMPLEX_SMALL, new Scalar(255,255,255));
            Imgproc.putText(mat, Double.toString(rot.thirdAngle), p3, 5, Imgproc.FONT_HERSHEY_COMPLEX_SMALL, new Scalar(255,255,255));

            Imgproc.putText(mat, Double.toString(detections.get(0).pose.x), p4, 5, Imgproc.FONT_HERSHEY_COMPLEX_SMALL, new Scalar(255,255,255));
            Imgproc.putText(mat, Double.toString(detections.get(0).pose.y), p5, 5, Imgproc.FONT_HERSHEY_COMPLEX_SMALL, new Scalar(255,255,255));
            Imgproc.putText(mat, Double.toString(detections.get(0).pose.z), p6, 5, Imgproc.FONT_HERSHEY_COMPLEX_SMALL, new Scalar(255,255,255));

            log.add(p1);
            log1.add(p2);
            log2.add(p3);
            log4.add(p4);
            log5.add(p5);
            log6.add(p6);
        }
        plot(mat,log,new Scalar(255,0,0));
        plot(mat,log1,new Scalar(255,255,0));
        plot(mat,log2,new Scalar(255,0,255));
        plot(mat,log4,new Scalar(0,0,255));
        plot(mat,log5,new Scalar(0,255,0));
        plot(mat,log6,new Scalar(255,255,255));

        if (reset || time.milliseconds() * 0.05 > 640) {time.reset();log.clear();log1.clear();log2.clear();log4.clear();log5.clear();log6.clear();}

        return mat;
    }

    public void plot(Mat input, ArrayList<Point> log, Scalar Color) {
        for (int i = 0; i < log.size()-1; i++) {
            if (log.get(i+1).y > log.get(i).y + 0.75*log.get(i).y) {
                log.remove(i+1);
            }
            if (i == 0 || log.get(log.size()-1).y > log.get(i).y + 0.75*log.get(i).y)
            Imgproc.line(input,log.get(i),log.get(i+1),Color,3);
//            Point pon = new Point(log.get(i).x, Math.abs(log.get(i).y-log.get(i+1).y));
//            Imgproc.line(input,pon,pon,Color,4);

        }
    }

}
