package org.firstinspires.ftc.teamcode.robotcorelib.math.filters;

public class KalmanFilter {

    private double estimate;
    private double variance;
    private final double initialEstimate;
    private final double deltaEstimate;
    private final double deltaEstimateInitial;
    private final double pNoise;
    int n = 0;

    public KalmanFilter(double deltaEstimate, double pNoise, double initialEstimate, double deltaEstimateInitial) {
        this.deltaEstimate = deltaEstimate;
        this.pNoise = pNoise;
        this.initialEstimate = initialEstimate;
        this.deltaEstimateInitial = deltaEstimateInitial;
    }

    public void run(double measurement) {
        if(n == 0) {
            double initialVariance = Math.pow(deltaEstimateInitial, 2);
            variance = initialVariance + pNoise;
            estimate = initialEstimate;
            n++;
        } else {
            double uncertainty = Math.pow(deltaEstimate, 2);
            double gain = variance / (variance + uncertainty);
            estimate = estimate + (gain * (measurement - estimate));
            variance = ((1 - gain) * variance) + pNoise;
        }
    }

    public double getEstimate() {
        return estimate;
    }


}