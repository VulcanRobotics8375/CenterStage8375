package org.firstinspires.ftc.teamcode.robotcorelib.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PathPoint {

    public double x;
    public double y;
    public double theta;
    public double speed;
    public double turnSpeed;
    public double lookahead;

    public PathPoint() {}

    public PathPoint(double x, double y, double theta, double speed, double turnSpeed, double lookahead) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.speed = speed;
        this.turnSpeed = turnSpeed;
        this.lookahead = lookahead;
    }


    public Point toPoint() {
        return new Point(x, y);
    }

    public Pose2d toPose2d() {
        return new Pose2d(x, y, theta);
    }

    public void setPathPoint(PathPoint point) {
        this.x = point.x;
        this.y = point.y;
        this.theta = point.theta;
        this.speed = point.speed;
        this.turnSpeed = point.turnSpeed;
        this.lookahead = point.lookahead;
    }

    public void setPoint(Point point) {
        this.x = point.x;
        this.y = point.y;
    }

}
