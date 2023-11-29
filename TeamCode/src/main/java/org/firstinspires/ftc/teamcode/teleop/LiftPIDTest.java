package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.math.control.PID;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@Config
@TeleOp(name = "LiftPIDTest")
public class LiftPIDTest extends OpModePipeline {
    MainConfig subsystems = new MainConfig();
    public static double P = 0.025, I=0.0008, D=0.01, f=0.1;
    PID miniPID = new PID(P, I, D, f);

    @Override
    public void init() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        miniPID.setOutputLimits(1.0);
        super.init();
    }

    public void loop() {
        Robot.update();

        miniPID.setPID(P, I, D, f);

        subsystems.lift.testPID(miniPID);
        telemetry.addData("P", P);
        telemetry.addData("D", I);
        telemetry.addData("D", D);
        telemetry.addData("f", f);

        telemetry.update();
    }
}