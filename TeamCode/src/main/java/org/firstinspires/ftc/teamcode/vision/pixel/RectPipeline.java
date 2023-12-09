package org.firstinspires.ftc.teamcode.vision.pixel;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class RectPipeline extends OpenCvPipeline {

    Scalar highHSV = new Scalar(180, 255, 255), lowHSV = new Scalar(150, 100, 100);

    public int rConstant = 23, lConstant = 16, cConstant = 1;

    Mat inputHSV = new Mat();
    Mat l, c, r;
    Mat lInRange = new Mat(), cInRange = new Mat(), rInRange = new Mat();
    public static double lx = 0.13, y = 12.0 / 16.0, sidew = 2.5 / 14.0 + 0.02, sideh = 4.0 / 16.0, cx = 5.5 / 16.0, cy = y - 1.0 / 16.0, cw = 1.0 - 2.0 * cx, ch = 3.0 / 16.0;
    int width = 1280, height = 720;
    Rect lRect = new Rect((int) (width * lx), (int) (height * y), (int) (width * sidew), (int) (height * sideh));
    Rect rRect = new Rect((int) (width * (1-lx - sidew)), (int) (height * y), (int) (width * sidew), (int) (height * sideh));
    Rect cRect = new Rect((int) (width * cx), (int) (height * cy), (int) (width * cw), (int) (height * ch));

    //    ArrayList<Integer> count = new ArrayList<>();
    public int lCount = 0, rCount = 0, cCount = 0;
    public int propIdx = 0;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, inputHSV, Imgproc.COLOR_RGB2HSV);

        l = inputHSV.submat(lRect);
        r = inputHSV.submat(rRect);
        c = inputHSV.submat(cRect);

        Imgproc.rectangle(input, lRect, new Scalar(255, 255, 255));
        Imgproc.rectangle(input, rRect, new Scalar(255, 255, 255));
        Imgproc.rectangle(input, cRect, new Scalar(255, 255, 255));

        Core.inRange(l, lowHSV, highHSV, lInRange);
        Core.inRange(r, lowHSV, highHSV, rInRange);
        Core.inRange(c, lowHSV, highHSV, cInRange);

        lCount = (int)Core.sumElems(lInRange).val[0] / 25500 - lConstant;
        rCount = (int)Core.sumElems(rInRange).val[0] / 25500 - rConstant;
        cCount = (int)Core.sumElems(cInRange).val[0] / 25500 - cConstant;

        if (lCount > cCount && lCount > rCount) { propIdx = 0; }
        else if (cCount > lCount && cCount > rCount) { propIdx = 1; }
        else if (rCount > cCount && rCount > lCount) { propIdx = 2; }
        // we could add to a list and take an average cuz we baddies %100
        else { propIdx = -1; }

        return input;
    }

    public void detectRed(boolean red) {
        if (red) {
            highHSV = new Scalar(180, 255, 255);
            lowHSV = new Scalar(150, 100, 100);
            rConstant = 23;
            lConstant = 16;
            cConstant = 1;
        } else {
            highHSV = new Scalar(140,255,255);
            lowHSV = new Scalar(100, 50, 50);
            rConstant = 50;
            lConstant = 39;
            cConstant = 33;
        }
    }

    public int getPropIdx() {
        return propIdx;
    }
}