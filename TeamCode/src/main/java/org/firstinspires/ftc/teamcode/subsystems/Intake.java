package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robotcorelib.math.filters.ExponentialMovingAverage;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;


public class Intake extends Subsystem {
    DcMotorEx intake;
    private boolean button = false;
    private double power = 0;
    private double SERVO_UP = 0.32,SERVO_DOWN = 0.02;
    private final double MULTIPLIER = 0.01;
    private ExponentialMovingAverage emaCurrent = new ExponentialMovingAverage(0, 0.95);
    public Servo leftServo,rightServo;
    public double servoPosLeft = 0.5, servoPosRight = 0.5;
    private double servoPos = SERVO_UP;

    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");
    }

    private void setServos(double input) {
        servoPosLeft += input;
//        servoPosRight -= input;
        leftServo.setPosition(servoPosLeft);
//        rightServo.setPosition(servoPosRight);
    }

    public void run(boolean button) {
        if(button && !this.button) {
            this.button = true;
            power = 1.0 - power;
            servoPos = (servoPos == SERVO_UP) ? SERVO_DOWN : SERVO_UP;
        } else if(!button && this.button) {
            this.button = false;
        }
        leftServo.setPosition(servoPos);
        intake.setPower(power);

//        emaCurrent.run(intake.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("intake current", Math.round(emaCurrent.getEstimate()*10.0)/10.0);
    }
    public void test(double stick, boolean button) {
        if(button && !this.button) {
            this.button = true;
            power = 1.0 - power;
        }
        else if(!button && this.button) { this.button = false; }
        intake.setPower(power);
        setServos(stick * MULTIPLIER);

        telemetry.addData("left intake servo pos: ", servoPosLeft);
//        telemetry.addData("right intake servo pos: ", servoPosRight);
    }





}