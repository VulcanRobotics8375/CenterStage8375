package org.firstinspires.ftc.teamcode.robotcorelib.motion.path;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.robotcorelib.math.utils.MathUtils;
import org.firstinspires.ftc.teamcode.robotcorelib.util.PathPoint;

import java.util.ArrayList;
import java.util.HashMap;

public class PathBuilder {

    private PathPoint lastPoint;

    private PathPoint startPoint;
    private PathPoint endPoint;
    private ArrayList<PathPoint> guidePoints = new ArrayList<>();
    private HashMap<PathPoint, Runnable> tasks = new HashMap<>();

    private double speed = 1;
    private double turnSpeed = 1;
    private double lookahead = 5;
    private boolean maintainHeading = false;

    public PathBuilder() {}

    public PathBuilder lineToConstantHeading(Pose2d start, Pose2d end) {
        startPoint = new PathPoint(start.getX(), start.getY(), start.getHeading(), speed, turnSpeed, lookahead);
        endPoint = new PathPoint(end.getX(), end.getY(), end.getHeading(), speed, turnSpeed, lookahead);
        lastPoint = endPoint;
        return this;
    }

    public PathBuilder lineToConstantHeading(Pose2d start, Pose2d end, Pose2d... guidePoints) {

        startPoint = new PathPoint(start.getX(), start.getY(), start.getHeading(), speed, turnSpeed, lookahead);
        for (Pose2d point : guidePoints) {
            this.guidePoints.add(new PathPoint(point.getX(), point.getY(), point.getHeading(), speed, turnSpeed, lookahead));
        }
        endPoint = new PathPoint(end.getX(), end.getY(), end.getHeading(), speed, turnSpeed, lookahead);
        lastPoint = endPoint;

        return this;
    }

    public PathBuilder lineTo(Pose2d start, Pose2d end) {
        double theta = MathUtils.fullAngleWrap(Math.atan2(end.getY() - start.getY(), end.getX() - start.getX()));
        startPoint = new PathPoint(start.getX(), start.getY(), theta, speed, turnSpeed, lookahead);
        endPoint = new PathPoint(end.getX(), end.getY(), theta, speed, turnSpeed, lookahead);
        lastPoint = endPoint;

        return this;
    }

    public PathBuilder start(Pose2d start) {
        startPoint = new PathPoint(start.getX(), start.getY(), start.getHeading(), speed, turnSpeed, lookahead);
        lastPoint = startPoint;
        return this;
    }

    public PathBuilder addGuidePoint(Pose2d point) {
        if(maintainHeading) {
            PathPoint pathPoint = new PathPoint(point.getX(), point.getY(), point.getHeading(), speed, turnSpeed, lookahead);
            guidePoints.add(pathPoint);
            lastPoint = pathPoint;
        }
        else {
            double theta = MathUtils.fullAngleWrap(Math.atan2(point.getY() - lastPoint.y, point.getX() - lastPoint.x));
            PathPoint pathPoint = new PathPoint(point.getX(), point.getY(), theta, speed, turnSpeed, lookahead);
            guidePoints.add(pathPoint);
            lastPoint = pathPoint;
        }
        return this;
    }

    public PathBuilder addTask(Runnable runnable) {
        tasks.put(lastPoint, runnable);
        return this;
    }

    public PathBuilder end(Pose2d end) {
        if(maintainHeading) {
            endPoint = new PathPoint(end.getX(), end.getY(), end.getHeading(), speed, turnSpeed, lookahead);
        } else {
            double theta = MathUtils.fullAngleWrap(Math.atan2(end.getY() - lastPoint.y, end.getX() - lastPoint.x));
            endPoint = new PathPoint(end.getX(), end.getY(), theta, speed, turnSpeed, lookahead);
        }
        lastPoint = endPoint;
        return this;
    }

    public PathBuilder speed(double speed) {
        this.speed = speed;
        return this;
    }

    public PathBuilder turnSpeed(double turnSpeed) {
        this.turnSpeed = turnSpeed;
        return this;
    }

    public PathBuilder lookahead(double lookahead) {
        this.lookahead = lookahead;
        return this;
    }

    public PathBuilder maintainHeading(boolean maintainHeading) {
        this.maintainHeading = maintainHeading;
        return this;
    }

    public Path build() {
        return new Path(this);
    }

    public PathPoint getStartPoint() {
        return startPoint;
    }

    public PathPoint getEndPoint() {
        return endPoint;
    }

    public HashMap<PathPoint, Runnable> getTasks() {
        return tasks;
    }

    public ArrayList<PathPoint> getGuidePoints() {
        return guidePoints;
    }


}
