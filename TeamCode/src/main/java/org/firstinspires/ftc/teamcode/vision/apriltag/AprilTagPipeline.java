package org.firstinspires.ftc.teamcode.vision.apriltag;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

class AprilTagPipeline extends OpenCvPipeline {
    // Notice this is declared as an instance variable (and re-used), not a local variable
    Mat submat;

    @Override
    public void init(Mat firstFrame)
    {
        submat = firstFrame;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // Because a submat is a persistent reference to a region of the parent buffer,
        // (which in this case is `input`) any changes to `input` will be reflected in
        // the submat (and vice versa).
        return input;
    }
}