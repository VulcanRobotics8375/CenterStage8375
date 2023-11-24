package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Switch;


public class Hopper extends Subsystem {
    boolean button1, button2;
    Switch aSwitch, bSwitch = new Switch();

    Servo leftServo, rightServo, pusherServo;
    Double LEFT_SERVO_MAX = 1.0,
            RIGHT_SERVO_MAX = 1.0,
            PUSHER_SERVO_MAX = 1.0;
    Double LEFT_SERVO_MIN = 0.0,
            RIGHT_SERVO_MIN = 0.0,
            PUSHER_SERVO_MIN = 0.0;

    Double lServo = 0.0,
            rServo = 0.0,
            pServo = 0.0;


    public void init() {
        leftServo = hardwareMap.get(Servo.class, "hopper_arm_left");
        lServo = leftServo.getPosition();
        rightServo = hardwareMap.get(Servo.class, "hopper_arm_right");
        rServo = rightServo.getPosition();
        pusherServo = hardwareMap.get(Servo.class, "hopper_pusher");
    }
    public void run() {}
    public void test(boolean button1, boolean button2, boolean button3) {
        if (aSwitch.simpleSwitch(button1)) {
            lServo = Range.clip(lServo + 0.05, LEFT_SERVO_MIN, LEFT_SERVO_MAX);
            rServo = Range.clip(rServo + 0.05, RIGHT_SERVO_MIN, RIGHT_SERVO_MAX);

        }
        if (bSwitch.simpleSwitch(button2)) {
            lServo = Range.clip(lServo - 0.05, LEFT_SERVO_MIN, LEFT_SERVO_MAX);
            rServo = Range.clip(rServo - 0.05, RIGHT_SERVO_MIN, RIGHT_SERVO_MAX);
        }


        leftServo.setPosition(lServo);
        rightServo.setPosition(rServo);

        telemetry.addData("lServo", lServo);
        telemetry.addData("rServo", rServo);

    }
}
