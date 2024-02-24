package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotcorelib.util.SubsystemState;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Toggle;


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

    public void init() {
        intake1 = hardwareMap.crservo.get("intake1");
        intake2 = hardwareMap.crservo.get("intake2");
        arm = hardwareMap.servo.get("intakeArm");
        v4barLeft = hardwareMap.servo.get("iV4barLeft");
        v4barRight = hardwareMap.servo.get("iV4barRight");
        v4barLeftEnc = hardwareMap.get(AnalogInput.class, "v4barLeftEnc");
        v4barRightEnc = hardwareMap.get(AnalogInput.class, "v4barRightEnc");
        door = hardwareMap.servo.get("door");
        extendoLeft = hardwareMap.get(DcMotorEx.class, "extendoLeft");
        extendoRight = hardwareMap.get(DcMotorEx.class, "extendoRight");

        extendoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extendoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        breakBeamFirst = hardwareMap.get(DigitalChannel.class, "breakBeamFirst");
        breakBeamFirst.setMode(DigitalChannel.Mode.INPUT);

        breakBeamSecond = hardwareMap.get(DigitalChannel.class, "breakBeamSecond");
        breakBeamSecond.setMode(DigitalChannel.Mode.INPUT);
    }

    public void intake() {
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
        armDown();
        v4barUp();
        stopIntake();
        doorClose();
    }

    public void transfer() {
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

    public boolean v4barIsUp() { return false; }

    public void armDown() {
        arm.setPosition(0.302);
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
    public void v4barDown() {}
    public void v4barHover() {}
    public void v4barUp() {}
    public void runIntake() {
        intake1.setPower(-1);
        intake2.setPower(1);
    }
    public void stopIntake() {
        intake1.setPower(0);
        intake2.setPower(0);
    }
    public void holdIntake() {
        intake1.setPower(0.1);
        intake2.setPower(0.1);
    }
    public void extendoIn() {}
    public void extendoOut() {}
}