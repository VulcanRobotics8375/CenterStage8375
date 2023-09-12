package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robotcorelib.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class MainConfig extends RobotConfig {

    public Drivetrain drivetrain;
    public Intake intake;

    public Flywheel flywheel;

    @Override
    public void init() {
        subsystems.clear();
        drivetrain = new Drivetrain();
        intake = new Intake();
        flywheel = new Flywheel();
    }
}