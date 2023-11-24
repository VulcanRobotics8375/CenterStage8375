package org.firstinspires.ftc.teamcode.vision.apriltag;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Collection;

public class AprilTagWrapper extends AprilTagDetectionPipeline {

    public AprilTagWrapper(double tagsize, double fx, double fy, double cx, double cy) {
        super(tagsize, fx, fy, cx, cy);
    }
    public Pose2d getPosition() {
        Double x = 0.0,y = 0.0,h = 0.0;
        ArrayList<AprilTagDetection> detections = getDetectionsUpdate();
        for (AprilTagDetection detection : detections) {
            Pose2d tag = aprilTagPositions[detection.id - 1];
            x += detection.pose.x - tag.getX();
            y += detection.pose.y - tag.getY();
//            h += detection.pose.R.get() - tag.getHeading();
        }

        return new Pose2d(x/detections.size(), y/detections.size());

    }

    public final Pose2d[] aprilTagPositions = new Pose2d[]{
            new Pose2d(0,0,0),
            new Pose2d(0,0,0),
            new Pose2d(0,0,0),
            new Pose2d(12.8125 + 4.5 + 3.625, 35, 90),
            new Pose2d(12.8125 + 4.5, 90),
            new Pose2d(12.8125 + 4.5 - 3.625, 35, 90),
    };
}
