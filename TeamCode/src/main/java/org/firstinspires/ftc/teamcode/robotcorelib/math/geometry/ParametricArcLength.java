package org.firstinspires.ftc.teamcode.robotcorelib.math.geometry;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.integration.SimpsonIntegrator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.util.FastMath;

public class ParametricArcLength implements UnivariateFunction {
    PolynomialSplineFunction xSpline;
    PolynomialSplineFunction ySpline;

    SimpsonIntegrator integrator = new SimpsonIntegrator();

    double lastCTEt;

    public ParametricArcLength(PolynomialSplineFunction x, PolynomialSplineFunction y) {
        this.xSpline = x;
        this.ySpline = y;
    }

    @Override
    public double value(double t) {
        if(t < lastCTEt) {
            return integrator.integrate(
                    1000,
                    i -> FastMath.sqrt(FastMath.pow(xSpline.derivative().value(i), 2) + FastMath.pow(ySpline.derivative().value(i), 2)),
                    t,
                    lastCTEt
            );
        } else if(t > lastCTEt) {
            return integrator.integrate(
                    1000,
                    i -> FastMath.sqrt(FastMath.pow(xSpline.derivative().value(i), 2) + FastMath.pow(ySpline.derivative().value(i), 2)),
                    lastCTEt,
                    t
            );
        } else {
            return 0;
        }
    }

    public void updateLastCTEt(double lastCTEt) {
        this.lastCTEt = lastCTEt;
    }
}