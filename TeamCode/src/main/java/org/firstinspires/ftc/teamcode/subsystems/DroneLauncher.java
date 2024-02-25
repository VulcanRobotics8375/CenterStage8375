package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotcorelib.util.SubsystemState;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Toggle;


public class DroneLauncher extends SubsystemState {
    public Servo trigger;
    private Toggle droneToggle = new Toggle();

    public void init() {
        trigger = hardwareMap.servo.get("trigger");
    }

    public void run() {

    }

    @Override
    public void intake() {
        run();
    }

    @Override
    public void deposit() {
        run();
    }

    @Override
    public void transfer() {
        run();
    }

    public void hold() {
        trigger.setPosition(0.4416);
    }
    public void fire() {
        trigger.setPosition(0.99);
    }
}