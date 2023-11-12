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
    private boolean up = false;
    private boolean down = false;
    private final double OPEN = 0.535;
    private final double CLOSED = 0.6208;
    private double servoPos = CLOSED;
    private final double closedLift = 50;

    public void init() {
        door = hardwareMap.servo.get("door");
    }

    public void run(boolean button) {

        if(button && !this.button) {
            this.button = true;
            servoPos = (servoPos == CLOSED) ? OPEN : CLOSED;
        } else if(!button && this.button) {
            this.button = false;
        }
        door.setPosition(servoPos);
    }
    public void test(boolean up, boolean down, double stick) {
        if(up && !this.up) {
            this.up = true;
            servoPos+=0.01;
        } else if(!up && this.up) {
            this.up = false;
        }
        if(down && !this.down) {
            this.down = true;
            servoPos-=0.01;
        } else if(!down && this.down) {
            this.down = false;
        }
        servoPos-=stick*0.005;
        door.setPosition(servoPos);

        telemetry.addData("servoPos", servoPos);
    }
}
