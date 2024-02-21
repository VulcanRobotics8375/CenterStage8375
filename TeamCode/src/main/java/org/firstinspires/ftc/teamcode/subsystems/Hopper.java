package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotcorelib.util.SubsystemState;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Toggle;


public class Hopper extends SubsystemState {
    public Servo door, paddle;
    public Servo leftArm, rightArm;
    public AnalogInput leftArmAnalog, rightArmAnalog;

    private boolean doorButton = false;
    private boolean paddleButton = false;
    private Toggle aToggle, bToggle = new Toggle();

    private double LEFT_ARM_UP = 0.7215;
    private double LEFT_ARM_DOWN = 0.5068;
    private double LEFT_ANALOG_DOWN = 1.175;
    private double LEFT_ANALOG_UP = 0.637;
    private double leftArmPos = 0.5;

    private double RIGHT_ARM_UP = 0.86;
    private double RIGHT_ARM_DOWN = 0.647;
    private double RIGHT_ANALOG_DOWN = 2.15;
    private double RIGHT_ANALOG_UP = 2.71;
    private double rightArmPos = 0.5;

    private double PADDLE_INSIDE = 0.79, PADDLE_PUSHED = 0.55637;

    private double paddlepos = 0.5, doorpos = 0.5;

    private double DOOR_CLOSED = 0.345, DOOR_OPENED = 0.426;

    private boolean liftReady = false;

    private Toggle mosaicToggle = new Toggle();
    private boolean mosaicing = false;
    private Toggle depoToggle = new Toggle();
    private boolean depositing = false;
    private ElapsedTime depoTimer = new ElapsedTime();

    private Toggle hopperToggle = new Toggle();
    private boolean hopperDown = false;

    public void init() {
        door = hardwareMap.get(Servo.class, "door");
        paddle = hardwareMap.get(Servo.class, "paddle");
        leftArm = hardwareMap.get(Servo.class, "hopper_arm_left");
        rightArm = hardwareMap.get(Servo.class, "hopper_arm_right");
        leftArmAnalog = hardwareMap.get(AnalogInput.class, "hopper_encoder_left");
        rightArmAnalog = hardwareMap.get(AnalogInput.class, "hopper_encoder_right");
    }

    @Override
    public void intake() {
        hopperDown();
        paddle.setPosition(PADDLE_INSIDE);
        door.setPosition(DOOR_CLOSED);
    }

    /*When button is pressed, (using a switch) we set the arm position to thirty degrees
    after that the paddle goes up to let the first pixel out for some amount of time
    after that, the paddle goes down and waits
    paddle goes up again and lets second pixel out, then paddle closes and arm position is reset
    */
    @Override
    public void deposit() {
        hopperUp();
        if (mosaicToggle.toggle(gamepad2.a) && !mosaicing && !depositing) { //Mosaic switch drops one at a time
            mosaicing = true;
            depoTimer.reset();
            door.setPosition(DOOR_OPENED);
            paddle.setPosition(PADDLE_PUSHED);
        }
        if (mosaicing && !depositing && depoTimer.milliseconds() > 700) {
            mosaicing = false;
            door.setPosition(DOOR_CLOSED);
            paddle.setPosition(PADDLE_INSIDE);
        }

        if (depoToggle.toggle(gamepad2.b) && !depositing && !mosaicing) { //Depo switch drops both pixels at the same time
            depositing = true;
            depoTimer.reset();
            door.setPosition(DOOR_OPENED);
            paddle.setPosition(PADDLE_INSIDE);
        }
        if (depositing && !mosaicing && depoTimer.milliseconds() > 800) {
            depositing = false;
            door.setPosition(DOOR_CLOSED);
            paddle.setPosition(PADDLE_INSIDE);
        }
    }

    @Override
    public void transfer() {
        hopperDown();
        paddle.setPosition(PADDLE_INSIDE);
        door.setPosition(DOOR_CLOSED);
    }

    public boolean intakeReady() { //The intake is ready when the arm is up
        return Math.abs(leftArmAnalog.getVoltage() - LEFT_ANALOG_DOWN) < 0.05 && Math.abs(rightArmAnalog.getVoltage() - RIGHT_ANALOG_DOWN) < 0.05;
    }

    public boolean depoReady() {
        return Math.abs(leftArmAnalog.getVoltage() - LEFT_ANALOG_UP) < 0.05 && Math.abs(rightArmAnalog.getVoltage() - RIGHT_ANALOG_UP) < 0.05;
    }

    public void hopperDown() {
        leftArm.setPosition(LEFT_ARM_DOWN);
        rightArm.setPosition(RIGHT_ARM_DOWN);
    }

    public void hopperUp() {
        leftArm.setPosition(LEFT_ARM_UP);
        rightArm.setPosition(RIGHT_ARM_UP);
    }

    public void doorClose() {
        door.setPosition(DOOR_CLOSED);
    }

    public void doorOpen() {
        door.setPosition(DOOR_OPENED);
    }

    public void testAnalog() {
        if(hopperToggle.toggle(gamepad2.y)) {
            hopperDown = !hopperDown;
        }
        if(hopperDown) {
            hopperDown();
        } else {
            hopperUp();
        }
        telemetry.addData("hopperLeftAnalog", leftArmAnalog.getVoltage());
        telemetry.addData("hopperRightAnalog", rightArmAnalog.getVoltage());
    }

    public void testLeft(double stick) {
        leftArmPos = Range.clip(leftArmPos + stick * 0.01, 0.0, 1.0);
        leftArm.setPosition(leftArmPos);
        telemetry.addData("leftArmPos", leftArmPos);
        telemetry.addData("hopperLeftAnalog", leftArmAnalog.getVoltage());
    }

    public void testRight(double stick) {
        rightArmPos = Range.clip(rightArmPos + stick * 0.01, 0.0, 1.0);
        rightArm.setPosition(rightArmPos);
        telemetry.addData("rightArmPos", rightArmPos);
        telemetry.addData("hopperRightAnalog", rightArmAnalog.getVoltage());
    }

    public void testHopperInternalServos(double stick, double stick2) {
        paddlepos = Range.clip(paddlepos + stick * 0.01, 0.0, 1.0);
        paddle.setPosition(paddlepos);
        doorpos = Range.clip(doorpos + stick2 * 0.01, 0.0, 1.0);
        door.setPosition(doorpos);
        telemetry.addData("paddle", paddlepos);
        telemetry.addData("door", doorpos);
    }
}
