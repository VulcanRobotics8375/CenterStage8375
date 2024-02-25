package org.firstinspires.ftc.teamcode.robotcorelib.util;

import com.acmerobotics.roadrunner.Pose2d;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.geometry.Vector;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;

public class Centripedal {
    PolynomialSplineFunction xD1, yD1, xD2, yD2;
    double x1,y1,x2,y2,r;
    double m = 1.0;

    ArrayList<Pose2d> buffer = new ArrayList<>();

    public Centripedal(PolynomialSplineFunction xSpline, PolynomialSplineFunction ySpline) {
        if (xSpline != null && ySpline != null) {
            xD1 = xSpline.polynomialSplineDerivative();
            yD1 = ySpline.polynomialSplineDerivative();
            xD2 = xSpline.polynomialSplineDerivative();
            yD2 = ySpline.polynomialSplineDerivative();
        }
    }

    public double autonomous(double t, double V) {
        x1 = xD1.value(t);
        y1 = yD1.value(t);
        x2 = xD2.value(t);
        y2 = yD2.value(t);
        r = (Math.pow(Math.sqrt((x1*x1) + (y1*y1)),3))/((x1*y2)-(y1*x2));
        return m * (V*V)/r;
    }

    public double teleop(Pose2d pose, double V) {
        buffer.add(pose);
        Pose2d b1 = buffer.get(0);
        Pose2d b2 = buffer.get(1);
        Pose2d b3 = buffer.get(2);
        x1 = b1.position.x - b2.position.x;
        x2 = x1 - (b2.position.x - b3.position.x);
        y1 = b1.position.y - b2.position.y;
        y2 = y1 - (b2.position.y - b3.position.y);
        buffer.remove(0);
        buffer.remove(1);
        buffer.remove(2);
        return m * (V*V)/r;

    }

}
