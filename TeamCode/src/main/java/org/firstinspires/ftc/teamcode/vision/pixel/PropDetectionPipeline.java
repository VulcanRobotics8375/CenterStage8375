package org.firstinspires.ftc.teamcode.vision.pixel;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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



    Scalar highHSV = new Scalar(5, 255, 255), lowHSV = new Scalar(0, 50, 50);
    Scalar highPIX = new Scalar(255, 50, 255), lowPIX = new Scalar(0, 0, 0);

    private Mat lmat = new Mat();

    public int propLocation = 0;

    double hScale = 2.0 / 3.0;
    double wScale = 2.0 / 3.0;



    private ArrayList<RotatedRect> find (List <MatOfPoint> points) {
        ArrayList<RotatedRect> rps = new ArrayList<RotatedRect>();
        for (MatOfPoint cnt : points) {
            RotatedRect rotrect = Imgproc.minAreaRect(new MatOfPoint2f(cnt.toArray()));
            if (rotrect.size.area() > 30){
                rps.add(rotrect);
            }
        }

        return rps;
    }
    private ArrayList<Rect> findR (List <MatOfPoint> points) {
        ArrayList<Rect> rps = new ArrayList<>();
        for (MatOfPoint cnt : points) {
            Rect rect = Imgproc.boundingRect(new MatOfPoint2f(cnt.toArray()));
            if (rect.area() > 15) {
                rps.add(rect);
            }
        }

        return rps;
    }

    private void drawRotatedRectange(Mat input, RotatedRect rotrect, Scalar color) {
        Point[] rpoints = new Point[4];
        rotrect.points(rpoints);
        for (int i = 0; i < 4; ++i) {
            Imgproc.line(input, rpoints[i], rpoints[(i + 1) % 4], color);
        }
//        Imgproc.circle(input, rotrect.center, 1, color, 1);

    }


    private List<MatOfPoint> contour(Mat input, Scalar highHSV, Scalar lowHSV) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(mat, lowHSV, highHSV, mat);
        List<MatOfPoint> points = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mat, points, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, points, -1, new Scalar(0,0,0));
        return points;
    }

    private RotatedRect sort (ArrayList<RotatedRect> input) {


        for (int i = 1; i < input.size(); i++) {
            int j = i - 1;
            while (j >= 0 && input.get(j).size.area() > input.get(i).size.area()) {
                input.set(j + 1, input.get(j));
                j--;
            }
            input.set(j+1, input.get(i));
        }

        return input.get(input.size()-1);
    }



    @Override
    public Mat processFrame(Mat input) {
//        Scalar white = new Scalar(255, 255, 255);
//        Scalar blue = new Scalar(0, 255, 255);
//        Point p1 = new Point((input.width() - (input.width() * wScale /2)), 0);
//        Point p2 = new Point((input.width() + (input.width() * wScale /2)), (input.height() * hScale));
//        RotatedRect prop = sort(find(contour(input, highHSV, lowHSV)));
//        ArrayList<RotatedRect> spikes = find(contour(input, highHSV, lowHSV));
        ArrayList<Rect> spik = findR(contour(input, highHSV, lowHSV));
        for (Rect spike : spik) {
            Imgproc.rectangle(input, spike, new Scalar(255,255,255));
        }
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