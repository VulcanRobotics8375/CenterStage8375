package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robotcorelib.math.filters.ExponentialMovingAverage;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;


public class Intake extends Subsystem {
    DcMotorEx intake;
    private boolean button = false;
    private double power = 0;
    private double MIN_SERVO,MAX_SERVO;
    private final double MULTIPLIER = 0.1;
    private ExponentialMovingAverage emaCurrent = new ExponentialMovingAverage(0, 0.95);
    public Servo leftServo,rightServo;
    public double servopos = 0.5;

    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "right_odo");
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");
    }

    private void setServos(double input) {
        leftServo.setPosition(servopos + input);
        rightServo.setPosition(servopos - input);
    }

    public void run(boolean button) {
        if(button && !this.button) {
            this.button = true;
            power = 0.7 - power;
        } else if(!button && this.button) {
            this.button = false;
        }
        intake.setPower(power);

        emaCurrent.run(intake.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("intake current", Math.round(emaCurrent.getEstimate()*10.0)/10.0);
    }
    public void test(double stick, boolean button) {
        if(button && !this.button) {
            this.button = true;
            power = 0.7 - power;
        }
        else if(!button && this.button) { this.button = false; }
        intake.setPower(power);
        setServos(stick * MULTIPLIER);
    }




}