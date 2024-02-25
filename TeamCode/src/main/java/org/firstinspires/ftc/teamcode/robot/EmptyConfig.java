package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robotcorelib.robot.RobotConfig;

public class EmptyConfig extends RobotConfig {

    @Override
    public void init() {
        subsystems.clear();
    }
}