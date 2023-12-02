package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotcorelib.math.filters.ExponentialMovingAverage;
import org.firstinspires.ftc.teamcode.robotcorelib.util.SubsystemState;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Switch;


public class Intake extends SubsystemState {

    private ElapsedTime time = new ElapsedTime();
    DcMotorEx intake;
    private double power = 0;
    private final double MULTIPLIER = 0.01;
    private ExponentialMovingAverage emaCurrent = new ExponentialMovingAverage(0, 0.95);

    private Switch aSwitch = new Switch(), bSwitch = new Switch();

    public CRServo counterRoller;

    public ServoImplEx leftServo,rightServo;
    public double servoPosLeft = 0.5, servoPosRight = 0.5;
    public boolean armDown = false;
    public final double leftArmUp = 0.731;
    public final double leftArmDown = 0.13;
    public final double rightArmUp = 0.241;
    public final double rightArmDown = 0.8579;

    double upperrange = 0.0;

    private boolean hopperReady = true;
    private boolean liftReady = true;

    private Switch intakeSwitch = new Switch();
    private Switch transferSwitch = new Switch();

    DigitalChannel breakBeam;
    private int breakCount = 0;
    private Switch breakSwitch = new Switch();
    private ElapsedTime breakBeamTimeOffset = new ElapsedTime();

    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        counterRoller = hardwareMap.get(CRServo.class, "counterroller");
        counterRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        breakBeam = hardwareMap.get(DigitalChannel.class, "break_beam");
        breakBeam.setMode(DigitalChannel.Mode.INPUT);

        leftServo = hardwareMap.get(ServoImplEx.class, "intake_arm_left");
        rightServo = hardwareMap.get(ServoImplEx.class, "intake_arm_right");
        PwmControl.PwmRange range = new PwmControl.PwmRange(500, 2500);
        leftServo.setPwmRange(range);
        rightServo.setPwmRange(range);
    }

    public void intake() {
        transferSwitch.simpleSwitch(false);
        if (intakeSwitch.simpleSwitch(true)) {
            breakCount = 0;
            power = 0.0;
            armDown = false;
        }

        if (breakSwitch.simpleSwitch(!breakBeam.getState())) {
            breakCount += 1;
            breakBeamTimeOffset.reset();
        }

        if (aSwitch.simpleSwitch(gamepad2.a)) {
            power = 1.0 - power;
            armDown = !armDown;

            if (!(hopperReady && liftReady)) {
                power = 0.0;
                armDown = false;
            }
        }

        if (armDown) {
            armDown();
        } else {
            armUp();
        }

        intake.setPower(power);
        counterRoller.setPower(power);
        telemetry.addData("hopper ready", hopperReady);
        telemetry.addData("lift ready", liftReady);
    }

    public void deposit() {
        transferSwitch.simpleSwitch(false);
        intakeSwitch.simpleSwitch(false);
        intake.setPower(0);
        counterRoller.setPower(0);
        armUp();
    }

    public void transfer() {
        intakeSwitch.simpleSwitch(false);
        if (transferSwitch.simpleSwitch(true)) {
            power = -1.0;
        }

        if(aSwitch.simpleSwitch(gamepad2.a)) {
            power = -1.0 - power;
        }
        intake.setPower(power);
        counterRoller.setPower(power);
        armUp();
    }

    public boolean intakingComplete() {
        return false;
    }

    public void resetBreakBeam() {
        breakCount = 0;
    }

    public void updateHopperLift(boolean hopperReady, boolean liftReady) {
        this.hopperReady = hopperReady;
        this.liftReady = liftReady;
    }


//     public void intake(boolean button) {
//         intake.setPower(button ? 1.0 : 0.0);
//         counterRoller.setPower(button ? 1.0 : 0.0);
//     }

    public void armUp() {
        leftServo.setPosition(leftArmUp);
        rightServo.setPosition(rightArmUp);
    }

    public void armDown() {
        leftServo.setPosition(leftArmDown);
        rightServo.setPosition(rightArmDown);
    }

    private void setServos(double input) {
        servoPosRight = Range.clip(servoPosRight + input, 0.0, 1.0);
        rightServo.setPosition(servoPosRight);
//        servoPosLeft = Range.clip(servoPosLeft - input, 0.0, 1.0);
//        leftServo.setPosition(servoPosLeft);
    }

    public void testRight(double input) {
        servoPosRight = Range.clip(servoPosRight + input*0.01, 0.0, 1.0);
        rightServo.setPosition(servoPosRight);
        telemetry.addData("right intake servo pos: ", servoPosRight);
    }

    public void testLeft(double input) {
        servoPosLeft = Range.clip(servoPosLeft + input*0.01, 0.0, 1.0);
        leftServo.setPosition(servoPosLeft);
        telemetry.addData("left intake servo pos: ", servoPosLeft);
    }

    public void test(boolean intakeButton, boolean counterRollButton, boolean outtakeButton, double stick) {
        intake.setPower(outtakeButton || intakeButton ? (intakeButton ? 1.0 : -1.0) : 0.0);
        counterRoller.setPower(outtakeButton ? (counterRollButton || intakeButton ? 1.0 : -1.0) : 0.0);
        setServos(stick * MULTIPLIER);
//        upperrange = Range.clip(upperrange + (stick2 * 5), 500, 2500);
//        telemetry.addData("left intake servo pos: ", servoPosLeft);
        telemetry.addData("right intake servo pos: ", servoPosRight);
    }

    public void breakBeamTelemetry() {
        telemetry.addData("breakbeam", breakBeam.getState());
    }
}