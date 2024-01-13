package org.firstinspires.ftc.teamcode.opmodes15to20;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms10to15.ProgrammingBoard8;
import org.firstinspires.ftc.teamcode.mechanisms15to20.MecanumDrive;

@TeleOp
public class SimpleMecanumDrive extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    IMU imu;

    @Override
    public void init() {
        drive.init(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(RevOrientation));
    }
    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
//        double rotate = gamepad1.right_stick_x;
        //rotate /= 2.0;

        double robotAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


        double snapX = gamepad1.right_stick_x;
        double snapY = gamepad1.right_stick_y;
        double theta = Math.atan2(snapY, snapX);
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        if (theta != robotAngle) {
            drive.setPowers()
        }




        if (gamepad1.right_stick_){ }


        drive.drive(forward, right, rotate);
    }

}
