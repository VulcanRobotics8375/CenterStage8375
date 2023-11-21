package org.firstinspires.ftc.teamcode.apriltagstester;

import org.opencv.core.*;
import org.opencv.imgproc.*;
import org.openftc.easyopencv.OpenCvPipeline;

public class GrayPipeline extends OpenCvPipeline {

    @Override
    public void init(Mat input) {

    }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);
        return input;
    }

    @Override
    public void onViewportTapped() {

    }

}
