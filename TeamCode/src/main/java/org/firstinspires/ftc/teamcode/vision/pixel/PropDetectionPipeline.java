package org.firstinspires.ftc.teamcode.vision.pixel;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.MatOfPoint;

import java.util.ArrayList;
import java.util.List;


public class PropDetectionPipeline extends OpenCvPipeline {
    Mat mat = new Mat();

//    public int indentifyLocation(Mat spike region) {
//
//    }

    @Override
    public Mat processFrame(Mat input) {
//        input = input.submat(new Rect(0, (input.height())/3, input.width(), (2 * input.height())/3));
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
//        Scalar highHSV = new Scalar(120, 255, 255), lowHSV = new Scalar(100, 50, 50);
        Scalar highHSV = new Scalar(5, 255, 255), lowHSV = new Scalar(0, 50, 50);
        Core.inRange(mat, lowHSV, highHSV, mat);
        List<MatOfPoint> points = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mat, points, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, points, -1, new Scalar(0,0,0));
        for (MatOfPoint cnt : points) {
            RotatedRect rotrect = Imgproc.minAreaRect(new MatOfPoint2f(cnt.toArray()));

//            Imgproc.putText(input, String.valueOf(rotrect.size.area()), new Point(rotrect.center.x, rotrect.center.y), Imgproc.FONT_HERSHEY_COMPLEX, 0.0005 * rotrect.size.area(), new Scalar(255, 255, 255));
            if (rotrect.size.area() > 300) {
                Point[] rpoints = new Point[4];
                rotrect.points(rpoints);
                for (int i = 0; i < 4; ++i) {
                    Imgproc.line(input, rpoints[i], rpoints[(i + 1) % 4], new Scalar(255, 255, 255));
                }
            }


        }
        return input;
    }
}