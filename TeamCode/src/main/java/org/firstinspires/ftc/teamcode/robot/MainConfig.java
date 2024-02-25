package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robotcorelib.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Hopper;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

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
    public Deposit deposit;

    @Override
    public void init() {
        subsystems.clear();
        drivetrain = new Drivetrain();
        intake = new Intake();
        deposit = new Deposit();
//        droneLauncher = new DroneLauncher();
    }

    public void intake() {
        drivetrain.intake();
//        intake.updateHopperLift(true, lift.intakeReady());
        intake.intake();
        deposit.intake();
//        droneLauncher.intake();
    }
    
    public void deposit() {
        drivetrain.deposit();
        intake.deposit();
        deposit.deposit();
//        droneLauncher.deposit();
    }

    public void transfer() {
        drivetrain.transfer();
        intake.transfer();
        deposit.transfer();
//        droneLauncher.transfer();
    }
}