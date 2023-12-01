package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;
import org.firstinspires.ftc.teamcode.robotcorelib.util.SubsystemState;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Switch;


public class DroneLauncher extends SubsystemState {
    public ServoImplEx lifter, trigger;

    private final double LAUNCHER_UP = 0.49;
    private final double LAUNCHER_DOWN = 0.074;
    private double launcherPos = LAUNCHER_DOWN;
    private Switch liftSwitch = new Switch();

    private double TRIGGER_HELD = 0.908;
    private double TRIGGER_RELEASED = 0.6;
    private double triggerPos = TRIGGER_HELD;
    private Switch triggerSwitch = new Switch();

    public void init() {
        lifter = hardwareMap.get(ServoImplEx.class, "launch_lift");
        trigger = hardwareMap.get(ServoImplEx.class, "launch_trigger");
        PwmControl.PwmRange range = new PwmControl.PwmRange(500, 2500);
        lifter.setPwmRange(range);
        trigger.setPwmRange(range);
    }

    public void run() {
        if(liftSwitch.simpleSwitch(gamepad2.right_bumper)) {
            launcherPos = launcherPos == LAUNCHER_DOWN ? LAUNCHER_UP : LAUNCHER_DOWN;
        }
        if (launcherPos == LAUNCHER_UP) {
            if (triggerSwitch.simpleSwitch(gamepad2.left_bumper)) {
                triggerPos = TRIGGER_RELEASED;
            }
        }
        lifter.setPosition(launcherPos);
        trigger.setPosition(triggerPos);
    }

    public void controlAngle(double stick) {
        launcherPos = Range.clip(launcherPos + stick*0.01, 0.0, 1.0);

        lifter.setPosition(launcherPos);

        telemetry.addData("launcher pos", launcherPos);
    }

    public void controlTrigger(double stick) {
        triggerPos = Range.clip(triggerPos + stick*0.1, 0.0, 1.0);

        trigger.setPosition(triggerPos);

        telemetry.addData("trigger pos", triggerPos);
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




}