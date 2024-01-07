package org.firstinspires.ftc.teamcode.robotcorelib.motion.followers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.robotcorelib.math.geometry.Vector;

public class centripedal {

    ElaspedTime time = new ElapsedTime();
    Pose2d previous = new Pose2d(0,0);
    Double lastDerivative = 0.0;



    public PolynomialSplineFunction xSpline;
    public PolynomialSplineFunction ySpline;

    public Double getCorrection(Pose2d current) {
        double derivative = (previous.getY() - current.getY()) / (current.getX() - previous.getX());
        double secondDerivative = (derivative - lastDerivative) / (current.getX() - previous.getX());
        return Math.sqrt(Math.pow(Math.pow(derivative, 2) + 1, 3)) / secondDerivative;
    }

    public double getParametricCurvature(double t ) {
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


}
