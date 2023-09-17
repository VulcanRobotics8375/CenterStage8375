package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class PracticeHopper extends Subsystem {
    Servo hopperServo;
    private final double SERVO_OPEN = 0.5;
    private final double SERVO_CLOSED = 0.0;
    private boolean previousOpened = false;
    private double SERVO_POSITION = 0.0;

    @Override
    public void init() {
       hopperServo = hardwareMap.get(Servo.class, "hopper servo");

    }
    public void run(boolean buttonA) {
        if (buttonA && !previousOpened) {
            SERVO_POSITION = (SERVO_POSITION==SERVO_OPEN) ? SERVO_CLOSED : SERVO_OPEN;
            previousOpened = true;
        }
        else if (!buttonA && previousOpeoned){
            previousOpened = false;
        }
        hopperServo.setPosition(SERVO_POSITION);

    }
}
