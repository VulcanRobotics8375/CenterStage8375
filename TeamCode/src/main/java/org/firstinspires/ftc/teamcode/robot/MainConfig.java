package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robotcorelib.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Hang;
import org.firstinspires.ftc.teamcode.subsystems.Hopper;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

/*
Control Hub

Motors

0 : front_right | lift
1 : back_right | rightEncoder
2 : front_left | frontEncoder
3 : back_left | leftEncoder

Servos

0 : hang_trigger
1 : launch_lift
3 : paddle
4 : counterroller
5 : intake_arm_left

Digital Devices

1 : break_beam

Analog Input Devices

0 : hopper_encoder_left
1 : hopper_encoder_right


Expansion Hub

Motors

0 : intake
1 : lift
2 : hanger

Servos

0 : intake_arm_right
1 : hang_arm
2 : launch_trigger
3 : hopper_arm_left
4 : hopper_arm_right
5 : door
*/

public class MainConfig extends RobotConfig {
    public Drivetrain drivetrain;
    public Intake intake;
    public Hopper hopper;
    public Lift lift;
    public DroneLauncher droneLauncher;

    @Override
    public void init() {
        subsystems.clear();
        drivetrain = new Drivetrain();
        intake = new Intake();
        hopper = new Hopper();
        lift = new Lift();
        droneLauncher = new DroneLauncher();
    }

    public void intake() {
        drivetrain.intake();
        intake.updateHopperLift(true, lift.intakeReady());
        intake.intake();
        lift.intake();
        hopper.intake();
        droneLauncher.intake();
    }
    
    public void deposit() {
        drivetrain.deposit();
        intake.deposit();
        lift.deposit();
        hopper.deposit();
        droneLauncher.deposit();
    }

    public void transfer() {
        drivetrain.transfer();
        intake.transfer();
        lift.transfer();
        hopper.transfer();
        droneLauncher.transfer();
    }
}