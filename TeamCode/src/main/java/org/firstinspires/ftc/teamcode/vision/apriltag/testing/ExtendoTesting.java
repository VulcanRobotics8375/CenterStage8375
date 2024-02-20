package org.firstinspires.ftc.teamcode.vision.apriltag.testing;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ExtendoTesting extends OpenCvPipeline {

    Scalar whiteLow = new Scalar(0, 0, 170);
    Scalar whiteHigh = new Scalar(63, 52, 255);

    Mat mask = new Mat();
    Mat mat= new Mat();

    ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    Mat hierarchy = new Mat();

    ArrayList<Rect> rects = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(mat, whiteLow, whiteHigh,mask);
        Imgproc.GaussianBlur(mask, mask, new Size(15, 15), 1, 1, 0);
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < 1000 && area > 100) {
                Rect rect = Imgproc.boundingRect(contour);
                Imgproc.rectangle(input, rect, new Scalar(255, 255, 255), 2);
                Imgproc.putText(input, "Yellow", rect.tl(), 0, 1, new Scalar(255, 255, 255), 1);

                rects.add(rect);
            }
        }
        contours.clear();
        return input;
    }
}
