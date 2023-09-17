package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;


public class PracticeLift extends Subsystem {

    private final int LEFT_LOW = 0;
    private final int RIGHT_LOW = 0;
    private final int LEFT_HIGH = 250;
    private final int RIGHT_HIGH = 250;
    private int leftPos = 0;
    private int rightPos = 0;

    private boolean previouslyPressed = false;



    DcMotorEx leftLift;
    DcMotorEx rightLift;


    public void init() {
        leftLift = hardwareMap.get(DcMotorEx.class, "left lift");
        rightLift = hardwareMap.get(DcMotorEx.class, "right lift");

        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void run(int leftStickY) {

        leftPos = Range.clip(leftPos + leftStickY, LEFT_LOW, LEFT_HIGH);
        rightPos = Range.clip(rightPos + leftStickY, RIGHT_LOW, RIGHT_HIGH);

        leftLift.setTargetPosition(leftPos);
        rightLift.setTargetPosition(rightPos);


    }

}
