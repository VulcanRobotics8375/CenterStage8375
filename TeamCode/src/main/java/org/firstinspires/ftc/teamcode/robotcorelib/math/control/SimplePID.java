package org.firstinspires.ftc.teamcode.robotcorelib.math.control;

import com.qualcomm.robotcore.util.Range;

public class SimplePID {

    private double Kp, Ki, Kd;

    private double integral;
    private double lastError;
    private double limMin, limMax;

    public SimplePID() {}

    public SimplePID(double Kp, double Ki, double Kd, double limMin, double limMax) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.limMin = limMin;
        this.limMax = limMax;
    }

    public double run(double target, double measurement) {
        return run(target - measurement);
    }

    public double run(double error) {

        double proportional = (Kp * error);
        integral += (error + lastError) / 2.0;
        double derivative = Kd * (error - lastError);

        double limMinInt, limMaxInt;
        limMaxInt = Math.min(proportional, limMax);
        limMinInt = Math.max(proportional, limMin);

        proportional = Range.clip(proportional, limMin, limMax);
        integral = Range.clip(integral, limMinInt, limMaxInt);

        lastError = error;
        return proportional + Ki * integral + derivative;
    }

    public double getLastError() {
        return lastError;
    }

    public double getIntegral() {
        return integral;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
    }
}
