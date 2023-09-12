package org.firstinspires.ftc.teamcode.robotcorelib.math.control;

public class PID {
    private double Kp, Ki, Kd;
    private double outputMaxLimit, outputMinLimit;
    private double error, integralError, lastError;
    private double integral, derivative;
    private double controllerOutput, controllerOutputB;
    private boolean isSaturating = false;
    private boolean integratorSaturate = false;
    private boolean integralClamp = false;

    public PID(double Kp, double Ki, double Kd, double outputMaxLimit, double outputMinLimit){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.outputMaxLimit = outputMaxLimit;
        this.outputMinLimit = outputMinLimit;
    }

    public double run(double target, double measurement){
        error = target - measurement;

        clampIntegral();

        integral += integralError;
        derivative = error - lastError;
        lastError = error;

        // Matt Attempt at Integral AntiWindup: Clamping
        controllerOutput = Kp * error + Ki * integral + Kd * derivative;
        controllerOutputB = controllerOutput;
        clampOutput();

        isSaturating = controllerOutput != controllerOutputB;

        dynamicClampIntegral();

        integratorSaturate = controllerOutputB * error > 0;
        integralClamp = isSaturating && integratorSaturate;

        return controllerOutput;
    }

    public void clampOutput() {
        if(controllerOutput > outputMaxLimit){
            controllerOutput = outputMaxLimit;
        } else if(controllerOutput < outputMinLimit){
            controllerOutput = outputMinLimit;
        }
    }

    public void dynamicClampIntegral() {
        if(Math.abs(Ki * integral) > Math.abs(Kp * error)){
            integral = (error * Kp)/Ki;
            controllerOutput = Kp * error + Ki * integral + Kd * derivative;
        }
    }

    public void clampIntegral() {
        if(!integralClamp){
            integralError = (error + lastError) / 2.0;
        }else{
            integralError = 0;
        }
    }

    public double getIntegral(){ return integral; }
    public double getIntegralError(){ return integralError; }
    public double getError() { return error; }
    public double getControllerOutput(){return controllerOutput; };
}
