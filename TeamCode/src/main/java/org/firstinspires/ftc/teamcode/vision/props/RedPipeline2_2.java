package org.firstinspires.ftc.teamcode.vision.props;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedPipeline2 extends OpenCvPipeline {

    Telemetry telemetry;

    public RedPipeline2(Telemetry t) {
        telemetry = t;
    } // constructor for the class to set up telemetry

    Mat mat = new Mat(), mat1 = new Mat(); // declare a new matrix (computer representation of an image)
    Rect leftRect = new Rect(140, 270, 200, 100); // define our regions of interest (where the algorithm is focusing on) as rectangles
    Rect midRect = new Rect(570, 260, 530, 100);
//    Rect rightRect = new Rect(1080, 230, 200, 117);
    final double PERCENT_THRESHOLD = 0.10; // define our threshold
    private int finalAnswer;
    Scalar red = new Scalar(255, 0, 0); // define what the color of the rectangle outline is that appears on the output (red)

    public enum Location {
        RIGHT,
        MIDDLE,
        LEFT
    }

    private Location location;

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); // change the color space from rgb to HSV (Hue, Saturation, Value)
        Scalar lowRedBound = new Scalar(0, 60, 60); // set lower and upper bounds for the color we want to recognize (red in this case)
        Scalar highRedBound = new Scalar(20, 500, 500);

        Scalar lowRedBound2 = new Scalar(160, 60, 60);
        Scalar highRedBound2 = new Scalar(180, 255, 255);
//        technically red is also found in the 160-180 hue range for HSV, but let's see if it'll work with just the hue range of 0-10
        Mat hold = new Mat();
        Core.inRange(mat, lowRedBound, highRedBound, hold);
        Core.inRange(mat, lowRedBound2, highRedBound2, mat1);
        Mat cone = new Mat();
        Core.add(hold,mat1, cone);
        // see which pixels are in our range, convert the pixels we're looking for into white, store it back to mat i think
        Mat left =cone.submat(leftRect); // create sub-matrices for our regions of interest
        Mat middle = cone.submat(midRect);
//        Mat right = mat.submat(rightRect);

        double leftValue = Core.sumElems(left).val[0] / leftRect.area() / 255; // get the percentage of white pixels that are present
        double midValue = Core.sumElems(middle).val[0] / midRect.area() / 255;
//        double rightValue = Core.sumElems(right).val[0] / rightRect.area() / 255;

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]); // display the raw values to the driver hub
        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
//        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);

        left.release();
//        right.release();
        middle.release();
        mat1.release();
        cone.release();
        hold.release();

//        Imgproc.cvtColor(cone, cone, Imgproc.COLOR_GRAY2RGB);
        Imgproc.rectangle(cone, leftRect, red, 2); // draw the rectangles on the output matrix
//        Imgproc.rectangle(mat, rightRect, red, 2);
        Imgproc.rectangle(cone, midRect, red, 2);

        if(leftValue > PERCENT_THRESHOLD || midValue > PERCENT_THRESHOLD){
            if(leftValue > midValue) {
                location = Location.LEFT;
                telemetry.addData("Location", "Left");
                Imgproc.rectangle(cone, leftRect, new Scalar(0, 255, 0), 2); // change the rectangle of the detected position to green
            } else {
                location = Location.MIDDLE;
                telemetry.addData("Location", "Middle");
                Imgproc.rectangle(cone, midRect, new Scalar(0, 255, 0), 2);
            }
        } else {
            location = Location.RIGHT;
            telemetry.addData("Location", "Right");
            Imgproc.rectangle(cone, new Rect(100, 100, 50, 50), new Scalar(0, 255, 0), 2);
        }

//        if (leftValue < PERCENT_THRESHOLD && midValue < PERCENT_THRESHOLD && rightValue < PERCENT_THRESHOLD) {
//            finalAnswer = 0;
//        } else {
//            if (leftValue > midValue && leftValue > rightValue) {
//                finalAnswer = 1;
//            } else if (midValue > leftValue && midValue > rightValue) {
//                finalAnswer = 2;
//            } else if (rightValue > leftValue && rightValue > midValue) {
//                finalAnswer = 3;
//            } else if (leftValue == midValue && leftValue == rightValue && midValue == rightValue) {
//                finalAnswer = 4;
//            } else {
//                finalAnswer = 5;
//            }
//        }
//
//        if (finalAnswer == 0) {
//            telemetry.addData("Position", "None of the values are above the percent threshold (please do more testing) - PROCEEDING WITH CAUTION ");
//            if (leftValue > midValue && leftValue > rightValue) {
//                telemetry.addData("Position", "left lol");
//                location = Location.LEFT;
//                Imgproc.rectangle(mat, leftRect, new Scalar(0, 255, 0), 2); // change the rectangle of the detected position to green
//            } else if (midValue > leftValue && midValue > rightValue) {
//                telemetry.addData("Position", "middle meh");
//                location = Location.MIDDLE;
//                Imgproc.rectangle(mat, midRect, new Scalar(0, 255, 0), 2);
//            } else if (rightValue > leftValue && rightValue > midValue) {
//                telemetry.addData("Position", "right rawr xD");
//                location = Location.RIGHT;
//                Imgproc.rectangle(mat, rightRect, new Scalar(0, 255, 0), 2);
//            }
//
//        } else if (finalAnswer == 1) {
//
//            telemetry.addData("Position", "left lol");
//            location = Location.LEFT;
//            Imgproc.rectangle(mat, leftRect, new Scalar(0, 255, 0), 2); // change the rectangle of the detected position to green
//
//        } else if (finalAnswer == 3) {
//
//            telemetry.addData("Position", "right rawr xD");
//            location = Location.RIGHT;
//            Imgproc.rectangle(mat, rightRect, new Scalar(0, 255, 0), 2);
//
//        } else if (finalAnswer == 2) {
//
//            telemetry.addData("Position", "middle meh");
//            location = Location.MIDDLE;
//            Imgproc.rectangle(mat, midRect, new Scalar(0, 255, 0), 2);
//
//        } else if (finalAnswer == 4) {
//
//            telemetry.addData("Debugging", "HOW TF DO THEY ALL EQUAL THE SAME");
//        } else if (finalAnswer == 5) {
//
//            telemetry.addData("Debugging", "Rethink and double check EVERYTHING");
//        }
//        telemetry.update();


        return mat;
    }

    public Location getLocation() {
        return location;
    }
}
