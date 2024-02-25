package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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
    AnalogInput v4barLeftEnc, v4barRightEnc;
    Servo depoFinger;
    ColorSensor colorSensorFirst, colorSensorLast;

    public PID liftPID = new PID(0.025,0.0008,0.01,0.1);
    PID xPID = new PID(0.0025,0.0,0.0,0.0), yPID = new PID(0.0025,0.0,0.0,0.0);
    private boolean liftHolding = false;
    private double minPos = 0;
    private final int LIFT_MAX_POS = 2800;
    private final double LIFT_CONVERGENCE_SPEED = 0.1;
    private final double LIFT_LIMIT_RANGE = 200.0;
    public double targetPos = 0.0;
    
    private Toggle halfFingers = new Toggle();
    private Toggle fullFingers = new Toggle();

    Point target = new Point(0,0);

    Vector stick = new Vector(0,0);

    public void init() {
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        v4barLeft = hardwareMap.servo.get("dV4barLeft");
        v4barRight = hardwareMap.servo.get("dV4barRight");

        v4barLeftEnc = hardwareMap.analogInput.get("dV4BarLEnc");
        v4barRightEnc = hardwareMap.analogInput.get("dV4BarREnc");

        depoFinger = hardwareMap.servo.get("depoFinger");

        colorSensorFirst = hardwareMap.colorSensor.get("cSensor1");
        colorSensorLast = hardwareMap.colorSensor.get("cSensor2");



        liftPID.setOutputLimits(1.0);
    }

    public void intake() {
        home();
    }

    public void deposit() {
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
                target = new Point(0, 1.5);
            }
        } else {
            target =  new Point(0, 0);
        }
        run2Axis(target);
    }

    public void transfer() {
        home();
    }
    public void home() {
        run2Axis(new Point(0,0));
        if (liftInMiddle()) {
                v4barIn();
                fingerClose();
        } else {
            v4barOut();
        }
    }

    public void updateGamepad(double stickX, double stickY, boolean fullFingerButton, boolean halfFingerButton) {
        stick = new Vector(stickX, stickY);
        fullFingers.toggle(fullFingerButton);
        halfFingers.toggle(halfFingerButton);
    }

    public boolean v4barHasClearance() { return false; }
    public boolean v4barIsOut() { return false; }
    public boolean liftHasClearance() { return false; }

    public boolean liftInMiddle(){
        double middle = (liftLeft.getCurrentPosition() - liftRight.getCurrentPosition())/2.0;
        return (Math.abs(middle)<20);
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
        liftLeft.setTargetPosition((int) (yPID.getOutput(y,My) + xPID.getOutput(x,Mx)));
        liftRight.setTargetPosition((int) (yPID.getOutput(y,My) - xPID.getOutput(x,Mx)));


    }
    public void v4barOut() {
        v4barLeft.setPosition(0.751);
        v4barRight.setPosition(0.225);
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
