package org.firstinspires.ftc.teamcode.robotcorelib.math.filters;

public class ExponentialMovingAverage {

    private double estimate;
    private double decaying_factor;

    public ExponentialMovingAverage(double initialEstimate, double decaying_factor) {
        this.estimate = initialEstimate;
        this.decaying_factor = decaying_factor;
    }

    public void run(double x) {
        this.estimate = decaying_factor * estimate + (1 - decaying_factor) * x;
    }

    public double getEstimate() {
        return estimate;
    }

    public void setEstimate(double estimate) {
        this.estimate = estimate;
    }

    public void setDecayingFactor(double decaying_factor) {
        this.decaying_factor = decaying_factor;
    }
}