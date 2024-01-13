package org.firstinspires.ftc.teamcode.mechanisms15to20;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TwoMotorDrive {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    public void init(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPowers(double leftPower, double rightPower) {
        double largest = 1.0;
        largest = Math.max(largest, Math.abs(leftPower));
        largest = Math.max(largest, Math.abs(rightPower));

        leftMotor.setPower(leftPower / largest);
        rightMotor.setPower(rightPower / largest);

    }
}
