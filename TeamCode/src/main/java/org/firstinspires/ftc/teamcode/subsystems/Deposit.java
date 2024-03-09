package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robotcorelib.math.control.PID;
import org.firstinspires.ftc.teamcode.robotcorelib.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Point;
import org.firstinspires.ftc.teamcode.robotcorelib.util.SubsystemState;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Toggle;
import org.firstinspires.ftc.teamcode.robotcorelib.util.hardware.Encoder;


public class Deposit extends SubsystemState {
    DcMotorEx liftLeft, liftRight;
    Servo v4barLeft, v4barRight;
//    AnalogInput v4barLeftEnc, v4barRightEnc;
    Servo depoFinger;
//    ColorSensor colorSensorFirst, colorSensorLast;

    private final int Y_MAX_POS = 2740;
    private final int X_MAX_POS = 590;
    
    private Toggle halfFingers = new Toggle();
    private Toggle fullFingers = new Toggle();

    Point target = new Point(0,0);

    Vector stick = new Vector(0,0);

    Toggle depositToggle = new Toggle();

    ElapsedTime v4barTimer = new ElapsedTime();

    public void init() {
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setTargetPosition(0);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setTargetPosition(0);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        v4barLeft = hardwareMap.servo.get("dV4barLeft");
        v4barRight = hardwareMap.servo.get("dV4barRight");

//        v4barLeftEnc = hardwareMap.analogInput.get("dV4BarLEnc");
//        v4barRightEnc = hardwareMap.analogInput.get("dV4BarREnc");

        depoFinger = hardwareMap.servo.get("depoFinger");
//
//        colorSensorFirst = hardwareMap.colorSensor.get("cSensor1");
//        colorSensorLast = hardwareMap.colorSensor.get("cSensor2");
    }

    public void intake() {
        depositToggle.toggle(false);
        home();
    }

    public void deposit() {
        depositToggle.toggle(true);
        if(depositToggle.getToggle()) {
            v4barTimer.reset();
        }
        v4barOut();
        if (v4barHasClearance()) {
            if (liftHasClearance()) {
                target = new Point(target.x + 0.01*stick.x, target.y + 0.01*stick.y);
                if (v4barIsOut()) {
                    if (fullFingers.getToggle()) {
                        fingerOpen();
                    } else if (halfFingers.getToggle()) {
                        dropOne();
                    }
                }
            } else {
                target = new Point(0, 0.07);
            }
        } else {
            target =  new Point(0, 0);
        }
        run2Axis(target);
    }

    public void transfer() {
        depositToggle.toggle(false);
        home();
    }
    public void home() {
        run2Axis(new Point(0,0));
        if (liftInMiddle()) {
                v4barIn();
        } else {
            v4barMiddle();
        }
        fingerClose();
    }

    public void updateGamepad(double stickX, double stickY, boolean fullFingerButton, boolean halfFingerButton) {
        stick = new Vector(stickX, stickY);
        fullFingers.toggle(fullFingerButton);
        halfFingers.toggle(halfFingerButton);
    }

    public boolean v4barHasClearance() { return v4barTimer.milliseconds() > 800; }
    public boolean v4barIsOut() { return v4barTimer.milliseconds() > 1100; }
    public boolean liftHasClearance() {
        return getY() > 0.05;
    }
    public boolean liftInMiddle() {
        return (Math.abs(getX()) < 0.03);
    }

    public double getY() {
        return (double)(liftLeft.getCurrentPosition() + liftRight.getCurrentPosition())/(2.0 * Y_MAX_POS);
    }

    public double getX() {
        return (double)(liftLeft.getCurrentPosition() - liftRight.getCurrentPosition())/(2.0 * X_MAX_POS);
    }

    public void run2Axis(Point target) {
        double y = (liftLeft.getCurrentPosition() + liftRight.getCurrentPosition())/2.0;
        double x = (liftLeft.getCurrentPosition() - liftRight.getCurrentPosition())/2.0;
        double My = target.y;
        double Mx = target.x;
        if (target.y < 20 && Math.abs(x) > 20) {
            Mx = 0;
            My = y;
        }
        liftLeft.setTargetPosition((int) (My + Mx));
        liftRight.setTargetPosition((int) (My - Mx));
        liftLeft.setPower(0.6);
        liftRight.setPower(0.6);
    }
    public void v4barOut() {
        v4barLeft.setPosition(0.751);
        v4barRight.setPosition(0.225);
    }
    public void v4barMiddle() {
        v4barLeft.setPosition(0.575);
        v4barRight.setPosition(0.394);
    }
    public void v4barIn() {
        v4barLeft.setPosition(0.1);
        v4barRight.setPosition(0.879);
    }
    public void fingerOpen() {
        depoFinger.setPosition(0.927);
    }
    public void fingerClose() {
        depoFinger.setPosition(0.7836);
    }
    public void dropOne() {
        depoFinger.setPosition(0.825);
    }
    public void testPID(PID xPID, PID yPID, Point target) { }
}
