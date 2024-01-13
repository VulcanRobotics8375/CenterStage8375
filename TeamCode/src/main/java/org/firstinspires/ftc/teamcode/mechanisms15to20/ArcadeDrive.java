package org.firstinspires.ftc.teamcode.mechanisms15to20;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class ArcadeDrive extends OpMode {
    TwoMotorDrive drive = new TwoMotorDrive();

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x / 2;


        if (gamepad1.a) {
            drive.setPowers(forward + right, forward - right);
        }
        else {
            drive.setPowers((forward + right) / 2.0, (forward - right) / 2.0);
        }
    }
}
