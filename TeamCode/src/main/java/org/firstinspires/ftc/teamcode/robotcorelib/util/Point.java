package org.firstinspires.ftc.teamcode.robotcorelib.util;

public class Point {
    public double x;
    public double y;

    public Point(){
        x = 0.0;
        y = 0.0;
    }

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point(Point p) {
        this.x = p.x;
        this.y = p.y;
    }

    public void setPoint(Point p) {
        this.x = p.x;
        this.y = p.y;
    }

}
