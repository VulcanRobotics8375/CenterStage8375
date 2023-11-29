package org.firstinspires.ftc.teamcode.robotcorelib.drive;

public interface DrivetrainImpl {

    //General Convention for writing setPowers() method:
    //4 motors(most drivetrains)-- front left, front right, back left, back right
    //if you have a diff configuration, add your new drive mode to DriveMode, and add update the kinematics for the path followers.

    void setPowers(double[] powers);

    DriveMode getDriveMode();
    DrivetrainVelocityMode getVelocityControlMode();

    //might not need more than this honestly. Really depends on what we need tho.

}
