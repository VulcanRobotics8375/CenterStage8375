package org.firstinspires.ftc.teamcode.robotcorelib.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotcorelib.drive.localization.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

import java.util.ArrayList;


//Configuration class for all subsystems to interface with the system properly.
public abstract class RobotConfig {

    /*
    HardwareMap Reference:
    MOTORS:
        Control Hub Ports:
            0 = front_left
            1 = front_right
            2 = back_left
            3 = back_right
        Expansion Hub Ports:
            0 = odo_left
            1 = odo_strafe
            2 = odo_right
            3 = lift
    SERVOS:
        Control Hub: NONE

        Expansion Hub:
            0 = release
            1 = cap
            4 = intake_wall
            5 = duck_servo

    I2C SENSORS:
        Control Hub: NONE

        Expansion Hub: NONE

    ANALOG PORTS:
        Control Hub: NONE

        Expansion Hub: NONE

     */

    public RobotConfig() {}

    public ArrayList<Subsystem> subsystems = new ArrayList<>();

    public HardwareMap hardwareMap;
    //localization
    public StandardTrackingWheelLocalizer localizer;

    //Subsystem objects go here
    //IMPORTANT:: Don't add more than one class that implements DrivetrainInterface, things will break

    //initialization of subsystems goes here
    public abstract void init();

}
