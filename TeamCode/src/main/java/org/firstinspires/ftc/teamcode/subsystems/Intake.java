package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robotcorelib.math.filters.ExponentialMovingAverage;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Switch;


public class Intake extends Subsystem {
    DcMotorEx intake;
    private double power = 0;
    private final double MULTIPLIER = 0.01;
    private ExponentialMovingAverage emaCurrent = new ExponentialMovingAverage(0, 0.95);

    private Switch aSwitch = new Switch();

    public CRServo counterRoller;

    public Servo leftServo,rightServo;
    public double servoPosLeft = 0.5, servoPosRight = 0.5;
    public boolean armDown = false;
    public final double leftArmUp = 0.73;
    public final double leftArmDown = 0.112;
    public final double rightArmUp = 0.01;
    public final double rightArmDown = 0.643;

    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        counterRoller = hardwareMap.get(CRServo.class, "counterroller");
        counterRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        leftServo = hardwareMap.get(Servo.class, "intake_arm_left");
        rightServo = hardwareMap.get(Servo.class, "intake_arm_right");
    }

    public void run(boolean button) {
        if (aSwitch.simpleSwitch(button)) {
            power = 1.0 - power;
            armDown = !armDown;
        }
        if(armDown){
            armDown();
        } else {
            armUp();
        }
        intake.setPower(power);
        counterRoller.setPower(power);

//        emaCurrent.run(intake.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("intake current", Math.round(emaCurrent.getEstimate()*10.0)/10.0);
    }

    public void intake(boolean button) {
        intake.setPower(button ? 1.0 : 0.0);
        counterRoller.setPower(button ? 1.0 : 0.0);
    }

    public void armUp() {
        leftServo.setPosition(leftArmUp);
        rightServo.setPosition(rightArmUp);
    }

    public void armDown() {
        leftServo.setPosition(leftArmDown);
        rightServo.setPosition(rightArmDown);
    }

    private void setServos(double input) {
//        servoPosRight = Range.clip(servoPosRight + input, 0.0, 1.0);
//        rightServo.setPosition(servoPosRight);
        servoPosLeft = Range.clip(servoPosLeft - input, 0.0, 1.0);
        leftServo.setPosition(servoPosLeft);
    }

    public void test(boolean intakeButton, boolean counterRollButton, boolean outtakeButton, double stick) {
        intake.setPower(outtakeButton || intakeButton ? (intakeButton ? 1.0 : -1.0) : 0.0);
        counterRoller.setPower(outtakeButton ? (counterRollButton || intakeButton ? 1.0 : -1.0) : 0.0);
        setServos(stick * MULTIPLIER);

        telemetry.addData("left intake servo pos: ", servoPosLeft);
//        telemetry.addData("right intake servo pos: ", servoPosRight);
    }





}