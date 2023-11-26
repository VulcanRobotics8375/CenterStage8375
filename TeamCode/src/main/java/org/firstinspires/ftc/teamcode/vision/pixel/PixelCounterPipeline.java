package org.firstinspires.ftc.teamcode.vision.pixel;


import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.A;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.MatOfPoint;

import java.nio.file.LinkPermission;
import java.util.ArrayList;
import java.util.List;


public class PixelCounterPipeline extends OpenCvPipeline {

    Scalar highPIX = new Scalar(255, 50, 255), lowPIX = new Scalar(0, 0, 0);

    private Mat lmat = new Mat();

    public int count = 0;

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

        int x = 100;
        int y = 100;
        int sx = 250;
        int sy = 250;
        int o = 50;
        para p = new para(
                new Point(x,y),
                new Point(x+sx,y),
                new Point(x+sx+o,y+sy),
                new Point(x+o,y+sy),
                input);

        count = p.fillCheck(100,100,input);

        Imgproc.putText(input, Integer.toString(count), new Point(0,300), Imgproc.FONT_HERSHEY_COMPLEX_SMALL, 4.0, new Scalar(255,255,255));
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

    //    public int getPropLocation() {
    //        return propLocation;
    //    }
}

class para {


//    Scalar highHSV = new Scalar(36, 255, 127), lowHSV = new Scalar(12, 12, 120);
    Scalar highHSV = new Scalar(180, 255, 255), lowHSV = new Scalar(150, 100, 100);
    Point p1,p2,p3,p4;

    double a, b, c, d;
    Mat img= new Mat();
    public para(Point p1, Point p2, Point p3, Point p4, Mat img) {
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
        this.p4 = p4;
        a = p2.x - p1.x;
        b = p2.y - p1.y;

        d = p4.x - p1.x;
        c = p4.y - p1.y;

        Imgproc.cvtColor(img,this.img,Imgproc.COLOR_RGB2HSV);
    }

    public int fillCheck(int i1, int i2, Mat input) {
        int count = 0;
        double x = p1.x, y=p1.y;
        for (int n=0; n < i1; n++) {
            x += d/i1;
            y += c/i1;
            double x2 = x;
            double y2 = y;
            for (int i=0; i < i2; i++) {
                x2 += a/i2;
                y2 += b/i2;

                if ( img.get((int)x2, (int)y2) != null) {
                    double[] pixel = img.get((int)y2, (int)x2);
//                    Imgproc.line(input, new Point((int)x2, (int)y2), new Point((int)x2, (int)y2),  new Scalar(0,0,0));
//                    Imgproc.putText(input, Double.toString(pixel[1]), new Point(x2,y2), Imgproc.FONT_HERSHEY_COMPLEX_SMALL, 0.4, new Scalar(255,255,255));
//                    Imgproc.putText(input, Double.toString(pixel[2]), new Point(x2,y2+5), Imgproc.FONT_HERSHEY_COMPLEX_SMALL, 0.4, new Scalar(255,255,255));
//                    Imgproc.putText(input, Double.toString(pixel[0]), new Point(x2,y2-5), Imgproc.FONT_HERSHEY_COMPLEX_SMALL, 0.4, new Scalar(255,255,255));


                    if (inRange(pixel, lowHSV, highHSV)) {
                        count ++;
                        Imgproc.line(input, new Point((int)x2, (int)y2), new Point((int)x2, (int)y2), new Scalar(255,255,255),1);

                    }
                }

            }

        }
        Scalar white = new Scalar(255,255,255);
        Imgproc.line(input, p1,p2, white);
        Imgproc.line(input,p2,p3,white);
        Imgproc.line(input,p3,p4,white);
        Imgproc.line(input,p4,p1,white);
        return count;
    }

    private boolean inRange(double[] pixel, Scalar lowHSV, Scalar highHSV) {
        if (pixel != null) {
            if (lowHSV.val[0] <= pixel[0] && pixel[0] <= highHSV.val[0]) {
                if (lowHSV.val[1] <= pixel[1] && pixel[1] <= highHSV.val[1]) {
                    if (lowHSV.val[2] <= pixel[2] && pixel[2] <= highHSV.val[2]) {
                        return true;
                    }
                }
            }
        }

        return false;
    }

}