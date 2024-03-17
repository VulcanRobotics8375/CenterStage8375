package org.firstinspires.ftc.teamcode.robotcorelib.util;

import org.apache.commons.math3.geometry.Vector;

public class Pose2d {
    double X = 0.0, Y = 0.0;
    double heading = 0.0;
    public Pose2d() {};
    public Pose2d(double X, double Y) {
        this.X = X;
        this.Y = Y;
    }
    public Pose2d(double X, double Y, double heading){
        this.X = X;
        this.Y = Y;
        this.heading = heading;
    };

    public double getX() {return X;}
    public double getY() {return Y;}
    public double getHeading() {return heading;}

}
