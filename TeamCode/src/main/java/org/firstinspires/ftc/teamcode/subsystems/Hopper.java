package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robotcorelib.math.control.SimplePID;
import org.firstinspires.ftc.teamcode.robotcorelib.math.filters.ExponentialMovingAverage;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;


public class Hopper extends Subsystem {
    Servo door;
    private boolean button = false;
    private final double OPEN = 0.69;
    private final double CLOSED = 0.232;
    private double servoPos = CLOSED;
    private final double closedLift = 50;

    public void init() {
        door = hardwareMap.servo.get("door");
    }

    public void run(boolean button, double liftPos) {

        if(button && !this.button) {
            this.button = true;
            servoPos = (servoPos == CLOSED) ? OPEN : CLOSED;
        } else if(!button && this.button) {
            this.button = false;
        }
        door.setPosition(servoPos);
    }
    public void test(double stick) {
        servoPos += stick*0.01;
        door.setPosition(servoPos);

        telemetry.addData("servoPos", servoPos);
    }
}
