package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robotcorelib.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DrivetrainConfig extends RobotConfig {
    public Drivetrain drivetrain;

    @Override
    public void init() {
        subsystems.clear();
        drivetrain = new Drivetrain();
    }
}