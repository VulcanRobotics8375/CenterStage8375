package org.firstinspires.ftc.teamcode.robotcorelib.math.geometry;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.util.FastMath;


public class Distance implements UnivariateFunction {

    UnivariateFunction function;
    Vector vec;

    public Distance(UnivariateFunction function) {
       this.function = function;
    }

    @Override
    public double value(double x) {
        return FastMath.hypot(vec.x - x, vec.y - function.value(x));
    }

    public void updatePos(Vector vec) {
        this.vec = vec;
    }
}