package org.firstinspires.ftc.teamcode.robotcorelib.math.geometry;

import org.apache.commons.math3.analysis.UnivariateFunction;

public class Derivative implements UnivariateFunction {
    UnivariateFunction function;
    double h = 0.0001;

    public Derivative(UnivariateFunction function) { this.function = function; }

    @Override
    public double value(double x) {
        return (function.value(x+h) - function.value(x-h)) / (2 * h);
    }
}
