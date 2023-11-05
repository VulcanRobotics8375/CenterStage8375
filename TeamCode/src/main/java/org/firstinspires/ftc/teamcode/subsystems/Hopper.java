package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Switch;


public class Hopper extends Subsystem {
    Servo door;
    private boolean button = false;
<<<<<<< Updated upstream
    private final double OPEN = 0.69;
    private final double CLOSED = 0.232;

    private final double LIFT_DOWN = 0.5;
=======

    private Switch servoSwitch = new Switch(button);
    private double OPEN = 1.0;
    private double CLOSED = 0.5;
>>>>>>> Stashed changes
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
        if (liftPos <= LIFT_DOWN) { servoPos = CLOSED; }
        door.setPosition(servoPos);
    }
    public void test(double stick) {

        if (CLOSED == 0.5 && servoSwitch.simpleSwitch(button)) { CLOSED = door.getPosition(); }
        if (CLOSED != 0.5  && servoSwitch.doubleClick(button) ) { OPEN = door.getPosition(); }

        servoPos += Range.clip(stick*0.01, 0 , 1);
        door.setPosition(servoPos);

        telemetry.addData("servoPos", servoPos);
    }
}
