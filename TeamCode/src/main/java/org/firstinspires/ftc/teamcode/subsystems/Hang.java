package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotcorelib.util.SubsystemState;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Switch;


public class Hang extends SubsystemState {
    public ServoImplEx hangRaise, trigger;
    public DcMotor hanger;

    private final double HANG_UP = 0.119;
    private final double HANG_DOWN = 0.471;
    private double hangPos = 0.5;
    private Switch hangSwitch = new Switch();

    private final double TRIGGER_HOME = 1.0;
    private final double TRIGGER_OPEN = 0.275;
    private double triggerPos = 0.5;
    private Switch triggerSwitch = new Switch();


    public void init() {
        hangRaise = hardwareMap.get(ServoImplEx.class, "hang_arm");
        trigger = hardwareMap.get(ServoImplEx.class, "hang_trigger");
        hanger = hardwareMap.get(DcMotor.class, "hanger");
//         PwmControl.PwmRange range = new PwmControl.PwmRange(500, 2500);
//         lifter.setPwmRange(range);
//         trigger.setPwmRange(range);
    }

    public void controlTrigger(double stick) {
        triggerPos = Range.clip(triggerPos + stick*0.01, 0.0, 1.0);

        trigger.setPosition(triggerPos);

        telemetry.addData("trigger pos", triggerPos);
    }
    
    public void run() {
        if(hangSwitch.simpleSwitch(gamepad1.right_bumper)) {
            hangPos = hangPos == HANG_DOWN ? HANG_UP : HANG_DOWN;
        }
        if (hangPos == HANG_UP) {
            if (triggerSwitch.simpleSwitch(gamepad1.left_bumper)) {
                triggerPos = triggerPos == TRIGGER_HOME ? TRIGGER_OPEN : TRIGGER_HOME;
            }
        } else {
            triggerPos = TRIGGER_HOME;
        }
        hangRaise.setPosition(hangPos);
        trigger.setPosition(triggerPos);

        if (gamepad1.a) {hanger.setPower(1);}
        else if (gamepad1.b) {hanger.setPower(-1);}
        else {hanger.setPower(0);}
    }

    public void intake(){
        run();
    }
    public void deposit(){
        run();
    }
    public void transfer(){
        run();
    }

    public void test(double stick, double stick2, boolean b1, boolean b2){
        hangPos = Range.clip(hangPos + stick*0.01, HANG_DOWN, HANG_UP);
        triggerPos = Range.clip(triggerPos + stick2*0.01, TRIGGER_OPEN, TRIGGER_HOME);

        if (b1) {hanger.setPower(1);}
        else if (b2) {hanger.setPower(-1);}
        else {hanger.setPower(0);}

        hangRaise.setPosition(hangPos);
        trigger.setPosition(triggerPos);

        telemetry.addData("hangerarmpos", hangPos);
        telemetry.addData("triggerpos", triggerPos);
    }
}