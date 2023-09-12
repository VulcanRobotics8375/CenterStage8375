package org.firstinspires.ftc.teamcode.robotcorelib.motion.followers;

import org.firstinspires.ftc.teamcode.robotcorelib.drive.DrivetrainImpl;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;

public abstract class Follower {

    DrivetrainImpl drivetrain;
    public Follower() {
        this.drivetrain = Robot.drivetrain;
    }

//    protected abstract void run();

}
