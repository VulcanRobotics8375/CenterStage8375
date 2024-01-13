package org.firstinspires.ftc.teamcode.robotcorelib.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.util.AutoTask;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;


public class AutoPipeline extends LinearOpMode {
    public RobotConfig subsystems;
    public RobotRunMode runMode;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void robotInit() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        if(subsystems != null && runMode != null) {
            Robot.init(this);
            if (runMode == RobotRunMode.TELEOP) {
                //teleop specific init
                telemetry.addLine("teleop mode");
            } else if (runMode == RobotRunMode.AUTONOMOUS) {
                //auton specific init
                telemetry.addLine("auton mode");
            }
        }
        else {
            telemetry.addLine("robot not initialized! RobotConfig or RobotRunMode does not exist");
        }
        telemetry.update();
    }

    protected void runTask(AutoTask task) {
        while(task.conditional() && !isStopRequested()) {
            Robot.update();
            task.run();
        }
    }


}
