package org.firstinspires.ftc.teamcode.robotcorelib.math.geometry;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.util.FastMath;

public class ParametricDistance implements UnivariateFunction {

    UnivariateFunction xSpline;
    UnivariateFunction ySpline;

    Vector vec;

    public ParametricDistance(UnivariateFunction x, UnivariateFunction y) {
        this.xSpline = x;
        this.ySpline = y;
    }

    @Override
    public double value(double t) {
        return FastMath.hypot(vec.x - xSpline.value(t), vec.y - ySpline.value(t));
    }

    public void updatePos(Vector vec) {
        this.vec = vec;
    }
}