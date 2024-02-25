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

    public static double xP = 0.0025, xI = 0.0, xD = 0.0, xf = 0.0;
    PID xPID = new PID(xP, xI, xD, xf);

    public static double yP = 0.0025, yI = 0.0, yD = 0.0, yf = 0.0;
    PID yPID = new PID(yP, yI, yD, yf);

    @Override
    public void init() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        xPID.setOutputLimits(1.0);
        yPID.setOutputLimits(1.0);
        super.init();
    }

    public void loop() {
        Robot.update();

        xPID.setPID(xP, xI, xD, xf);

//        subsystems.lift.testPID(xPID);
        telemetry.addData("P", xP);
        telemetry.addData("D", xI);
        telemetry.addData("D", xD);
        telemetry.addData("f", xf);

        telemetry.update();
    }
}