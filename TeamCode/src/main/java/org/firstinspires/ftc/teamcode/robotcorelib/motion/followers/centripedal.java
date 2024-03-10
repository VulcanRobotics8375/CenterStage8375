package org.firstinspires.ftc.teamcode.robotcorelib.motion.followers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.robotcorelib.math.geometry.Vector;

import java.util.ArrayList;

public class centripedal {

    ElapsedTime time = new ElapsedTime();
    Pose2d previous = new Pose2d(0,0);
    Double lastDerivative = 1.0;

    ArrayList<Vector> buffa = new ArrayList<>();



    public PolynomialSplineFunction xSpline;
    public PolynomialSplineFunction ySpline;

    public Double getCorrection(Pose2d current) {
        double derivative = (previous.getY() - current.getY()) / (current.getX() - previous.getX());
        double secondDerivative = (derivative - lastDerivative) / (current.getX() - previous.getX());
        return Math.sqrt(Math.pow(Math.pow(derivative, 2) + 1, 3)) / secondDerivative;
    }

    public double auto(double t) {
        PolynomialSplineFunction xD = xSpline.polynomialSplineDerivative();
        PolynomialSplineFunction yD = ySpline.polynomialSplineDerivative();

        double dx = xD.value(t);
        double dy = yD.value(t);

        double dx2 = xD.derivative().value(t);
        double dy2 = yD.derivative().value(t);

        double r = (FastMath.sqrt(FastMath.pow(FastMath.pow(dx, 2) + FastMath.pow(dy, 2), 3))) / ((dx * dy2) - (dy * dx2));
        return 1/r;
//       it is also important to understand why this works with a circle given the example by wolfpack movement breakdown
//        https://en.wikipedia.org/wiki/Radius_of_curvature for a really good demonstration as well
    }

    public double tele(Vector pose){
        buffa.add(pose);
        if (buffa.size() < 3) {
            return 0.0;
        }
        Vector pose1 = buffa.get(0);
        Vector pose2 = buffa.get(1);
        Vector pose3 = buffa.get(2);
        buffa.remove(0);
        buffa.remove(1);
        buffa.remove(2);

//        derive by x, dx/dx = 1, dy/dx is only thing we care about
        double dy = (pose1.y-pose2.y)/(pose1.x-pose2.x);
        double dy2 = dy-lastDerivative/(pose1.x-pose2.x);
        lastDerivative = dy;
        double r = (FastMath.sqrt(FastMath.pow((1+dy),3)))/dy2;
        return 1/r;
    }


}
