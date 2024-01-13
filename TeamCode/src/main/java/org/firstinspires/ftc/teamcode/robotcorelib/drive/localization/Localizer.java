package org.firstinspires.ftc.teamcode.robotcorelib.drive.localization;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public interface Localizer {

    void update();

    Pose2d getPose();
    Pose2d getVelocity();

    void setPose(Pose2d pose);

}
