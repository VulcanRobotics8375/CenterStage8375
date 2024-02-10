package org.firstinspires.ftc.teamcode.robotcorelib.util;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.geometry.Vector;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class Centripedal {
    PolynomialSplineFunction xD1, yD1, xD2, yD2;
    double x1,y1,x2,y2,r;
    double m = 1.0;

    public Centripedal(PolynomialSplineFunction xSpline, PolynomialSplineFunction ySpline) {
        xD1 = xSpline.polynomialSplineDerivative();
        yD1 = ySpline.polynomialSplineDerivative();
        xD2 = xSpline.polynomialSplineDerivative();
        yD2 = ySpline.polynomialSplineDerivative();
    }

    public double autonomous(double t, double V) {
        x1 = xD1.value(t);
        y1 = yD1.value(t);
        x2 = xD2.value(t);
        y2 = yD2.value(t);
        r = (Math.pow(Math.sqrt((x1*x1) + (y1*y1)),3))/((x1*y2)-(y1*x2));
        return m * (V*V)/r;
    }

}
