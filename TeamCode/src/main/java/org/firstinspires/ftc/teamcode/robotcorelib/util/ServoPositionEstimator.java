package org.firstinspires.ftc.teamcode.robotcorelib.util;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ServoPositionEstimator {

    private Servo servo;
    private double lastPos, targetPos;
    private ElapsedTime timer = new ElapsedTime();
    private double posEstimate;
    // velocity units are arbitrary, its just servo position / second
    // v=1 would mean the servo travels from position 0.0 to position 1.0 in 1 second
    private double velocity;

    public ServoPositionEstimator(Servo servo, double velocity) {
        this.servo = servo;
        lastPos = servo.getPosition();
        targetPos = servo.getPosition();
        this.velocity = velocity;
    }

    public void update() {
        if(servo.getPosition() != targetPos) {
            lastPos = targetPos;
            targetPos = servo.getPosition();
            timer.reset();
        }

       posEstimate = Range.clip(lastPos + (timer.seconds() * velocity), Math.min(lastPos, targetPos), Math.max(lastPos, targetPos));
    }

    public double getPosEstimate() {
        return posEstimate;
    }

}
