package org.firstinspires.ftc.teamcode.subsystems;
import android.util.Pair;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotcorelib.util.SubsystemState;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Toggle;
import org.opencv.core.Scalar;


public class Intake extends SubsystemState {
    CRServo intake1, intake2;
    Servo arm;
    Servo v4barLeft, v4barRight;
    AnalogInput v4barLeftEnc, v4barRightEnc;
    Servo door;
    DcMotorEx extendoLeft, extendoRight;
    DigitalChannel breakBeamFirst;
    DigitalChannel breakBeamSecond;

    private Toggle intakeToggle = new Toggle();
    private boolean intaking = false;
    private Toggle extendoToggle = new Toggle();
    private boolean extendoOut = false;
    private boolean depoTransferReady = true;

    private ElapsedTime transferTimer = new ElapsedTime();
    private Toggle transferToggle = new Toggle();


    ElapsedTime timer;

    ColorSensor firstColor, secondColor;

    boolean pixels= false;

    public void init() {
        intake1 = hardwareMap.crservo.get("intake1");
        intake2 = hardwareMap.crservo.get("intake2");
        arm = hardwareMap.servo.get("intakeArm");
        v4barLeft = hardwareMap.servo.get("iV4barLeft");
        v4barRight = hardwareMap.servo.get("iV4barRight");
//        v4barLeftEnc = hardwareMap.get(AnalogInput.class, "v4barLeftEnc");
//        v4barRightEnc = hardwareMap.get(AnalogInput.class, "v4barRightEnc");
        door = hardwareMap.servo.get("door");
        extendoLeft = hardwareMap.get(DcMotorEx.class, "extendoLeft");
        extendoRight = hardwareMap.get(DcMotorEx.class, "extendoRight");

//        firstColor = hardwareMap.colorSensor.get("Color1");
//        secondColor = hardwareMap.colorSensor.get("Color2");

        extendoLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        extendoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoLeft.setTargetPosition(0);
        extendoLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extendoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoRight.setTargetPosition(0);
        extendoRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        breakBeamFirst = hardwareMap.get(DigitalChannel.class, "breakBeamFirst");
//        breakBeamFirst.setMode(DigitalChannel.Mode.INPUT);
//
//        breakBeamSecond = hardwareMap.get(DigitalChannel.class, "breakBeamSecond");
//        breakBeamSecond.setMode(DigitalChannel.Mode.INPUT);
//        timer.startTime();
    }

    public void intake() {
        transferToggle.toggle(false);
        armDown();
        doorClose();
        if (intakeToggle.getToggle()) {
            intaking = !intaking;
            extendoOut = false;
        } else if (extendoToggle.getToggle()) {
            if (intaking) {
                extendoOut = !extendoOut;
            } else {
                intaking = true;
                extendoOut = true;
            }
        }
        if (intaking) {
            v4barDown();
            runIntake();
        } else {
            v4barHover();
            stopIntake();
        }
        if (extendoOut) {
            extendoOut();
        } else {
            extendoIn();
        }
    }

    public void deposit() {
        transferToggle.toggle(false);
        armDown();
        v4barUp();
        stopIntake();
        doorClose();
    }

    public void transfer() {
        transferToggle.toggle(true);
        if (transferToggle.getToggle()) {
            transferTimer.reset();
        }
        armDown();
        v4barUp();
        if(transferReady()) {
            doorOpen();
            runIntake();
        } else {
            doorClose();
            holdIntake();
        }
    }

//    public Pair<String, String> getPixels() {
//        Scalar c1= new Scalar(firstColor.red(), firstColor.green(), firstColor.blue());
//        Scalar c2 = new Scalar(secondColor.red(),secondColor.green(), secondColor.blue());
//    }
//    public Pair<Boolean,Boolean> getBreaks() {
//        return new Pair<>(breakBeamFirst.getState(), breakBeamSecond.getState());
//    }
//    private boolean inRange(Scalar c, Scalar lowHSV, Scalar highHSV) {
//        if (c != null) {
//            if (lowHSV.val[0] <= c.val[0] && c.val[0] <= highHSV.val[0]) {
//                if (lowHSV.val[1] <= c.val[1] && c.val[1] <= highHSV.val[1]) {
//                    return lowHSV.val[2] <= c.val[2] && c.val[2] <= highHSV.val[2];
//                }
//            }
//        }
//        return false;
//    }
//    private Scalar rgb_to_hsv(double r, double g, double b)
//    {
//        r = r / 255.0;
//        g = g / 255.0;
//        b = b / 255.0;
//
//        // h, s, v = hue, saturation, value
//        double cmax = Math.max(r, Math.max(g, b)); // maximum of r, g, b
//        double cmin = Math.min(r, Math.min(g, b)); // minimum of r, g, b
//        double diff = cmax - cmin; // diff of cmax and cmin.
//        double h = -1, s = -1;
//        if (cmax == cmin)
//            h = 0;
//        else if (cmax == r)
//            h = (60 * ((g - b) / diff) + 180) % 180;
//        else if (cmax == g)
//            h = (60 * ((b - r) / diff) + 60) % 180;
//        else if (cmax == b)
//            h = (60 * ((r - g) / diff) + 120) % 180;
//        if (cmax == 0)
//            s = 0;
//        else
//            s = (diff / cmax) * 100;
//        double v = cmax * 100;
//        return new Scalar(h,s,v);
//    }


    public void updateGamepad(boolean intake, boolean extend) {
        intakeToggle.toggle(intake);
        extendoToggle.toggle(extend);
    }

    public void updateSubsystems(boolean depoTransferReady) {
        this.depoTransferReady = depoTransferReady;
    }

    public boolean transferReady() {
        return v4barIsUp() && depoTransferReady;
    }

    public boolean v4barIsUp() { return transferTimer.milliseconds() > 1500; }

    public void armDown() {
        arm.setPosition(0.47);
    }
    public void armUp() {
        arm.setPosition(0.99);
    }
    public void doorClose() {
        door.setPosition(0.282);
    }
    public void doorOpen() {
        door.setPosition(0.1528);
    }
    public void v4barDown() {
        v4barLeft.setPosition(0.4394);
        v4barRight.setPosition(0.987);

    }
    public void v4barHover() {
        v4barLeft.setPosition(0.7678);
        v4barRight.setPosition(0.3355);
    }
    public void v4barUp() {
        v4barLeft.setPosition(0.849);
        v4barRight.setPosition(0.097);
    }
    public void runIntake() {
        intake1.setPower(-1);
        intake2.setPower(1);
    }
    public void reverseIntake() {
        intake1.setPower(1);
        intake2.setPower(-1);
    }
    public void stopIntake() {
        intake1.setPower(0);
        intake2.setPower(0);
    }
    public void holdIntake() {
        intake1.setPower(0.1);
        intake2.setPower(0.1);
    }

    public void extendoIn() {
        extendoLeft.setTargetPosition(0);
        extendoLeft.setPower(1);
        extendoRight.setTargetPosition(0);
        extendoRight.setPower(1);
    }
    public void extendoOut() {
        extendoLeft.setTargetPosition(400);
        extendoLeft.setPower(1);
        extendoRight.setTargetPosition(400);
        extendoRight.setPower(1);
    }
    public void extendoTo(int pos) {

    }
}