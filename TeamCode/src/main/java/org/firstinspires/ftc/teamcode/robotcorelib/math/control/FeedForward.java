package org.firstinspires.ftc.teamcode.robotcorelib.math.control;

public class FeedForward {
    private double Ka, Kv;
//    private double currentPosition, previousPosition;
//    private double currentVelocity, currentVelocityError, previousVelocity, desiredVelocity;
//    private double currentAcceleration, currentAccelerationError, desiredAcceleration;
//    private double previousError, currentError;
//    private double previousTime, currentTime;
    private double output;
    private double voltage;
    private double previousVelocity;
    private double currentVelocity;
    private double currentAcceleration;

    public FeedForward(double Kv, double Ka, double voltage, double currentVelocity){
        this.Kv = Kv;
        this.Ka = Ka;
        this.currentVelocity = currentVelocity;
    }

    public double run(){
//        currentTime = System.currentTimeMillis();
//
//        currentVelocity = (currentPosition - previousPosition) / (currentTime - previousTime);
//        currentVelocityError = desiredVelocity - currentVelocity;
//
//        currentAcceleration = (currentVelocity - previousVelocity) / (currentTime - previousTime);
//        currentAccelerationError = desiredAcceleration - currentAcceleration;

        currentAcceleration = currentVelocity - previousVelocity;
        output = (Kv * currentVelocity + Ka * currentAcceleration) / voltage;

//        previousTime = currentTime;
//        previousError = currentError;
//        previousPosition = currentPosition;

        previousVelocity = currentVelocity;
        return output;
    }
}
