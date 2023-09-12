package org.firstinspires.ftc.teamcode.robotcorelib.drive;

public enum DrivetrainVelocityMode {
    DRIVE_MOTOR_ENCODERS,
    //feedforward is not currently supported in pure pursuit, robot wont move in this mode
    FEEDFORWARD,
    PID

}
