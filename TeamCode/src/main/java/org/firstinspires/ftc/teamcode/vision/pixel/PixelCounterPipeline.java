package org.firstinspires.ftc.teamcode.vision.pixel;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.MatOfPoint;

import java.util.ArrayList;
import java.util.List;


public class PixelCounterPipeline extends OpenCvPipeline {
    Scalar highHSV = new Scalar(5, 255, 255), lowHSV = new Scalar(0, 50, 50);
    Scalar highPIX = new Scalar(255, 50, 255), lowPIX = new Scalar(0, 0, 0);

    private Mat lmat = new Mat();

    public int propLocation = 0;

    double hScale = 2.0 / 3.0;
    double wScale = 2.0 / 3.0;



//    private void drawParallelogram(Mat input, MatOfPoin rotrect, Scalar color) {
//        Point[] rpoints = new Point[4];
//        rotrect.points(rpoints);
//        for (int i = 0; i < 4; ++i) {
//            Imgproc.line(input, rpoints[i], rpoints[(i + 1) % 4], color);
//        }
////        Imgproc.circle(input, rotrect.center, 1, color, 1);
//
//    }



    @Override
    public Mat processFrame(Mat input) {


//        drawRotatedRectange(input, prop, new Scalar(0, 255, 255));
//
//
//
//
//
//        if (prop.center.x < input.width()/3.0) { propLocation = 1; }
//        if (prop.center.x > (2 * input.width())/3.0) { propLocation = 3; }
//        else { propLocation = 2; }


        return input;
    }

    public int getPropLocation() {
        return propLocation;
    }
}

class para {
    Point p1,p2,p3,p4;
    double s1, s2;
    Mat img;
    public para(Point p1, Point p2, Point p3, Point p4, Mat img) {
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
        this.p4 = p4;
        this.img = img;
        s1 = (p1.y - p2.y) / (p1.x - p2.x);
        s2 = (p2.y - p3.y) / (p2.x - p3.x);
    }

    public int fillCheck() {
        int count = 0;

    }

}