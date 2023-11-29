package org.firstinspires.ftc.teamcode.robotcorelib.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.util.AutoTask;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

public abstract class OpModePipeline extends OpMode {
    public RobotConfig subsystems;
    public RobotRunMode runMode;

    private volatile boolean stopRequested;

    @Override
    public void init() {
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

    public void start() {
        resetRuntime();
    }

    public abstract void loop();


    public void stop() {
        stopRequested = true;
        Robot.stop();
    }

    public boolean isStopRequested() {
        return stopRequested;
    }

    protected void runTask(AutoTask task) {
        while(task.conditional() && !isStopRequested()) {
            Robot.update();
            task.run();
        }
    }

    protected synchronized void maintainConstantLoopTime(int targetLoopNum, double elapsedTime) {
        synchronized (this) {
            double targetLoopTimeMs = 1000.0 / targetLoopNum;

            long maxWaitTimeMs = (long) Math.floor(targetLoopTimeMs);
            int maxWaitTimeNs = (int) ((targetLoopTimeMs - maxWaitTimeMs) * 100000);

            long elapsedTimeMs = (long) Math.floor(elapsedTime);
            int elapsedTimeNs = (int) ((elapsedTime - elapsedTimeMs) * 100000);

            if (maxWaitTimeMs > elapsedTimeMs || maxWaitTimeMs - elapsedTimeMs > targetLoopTimeMs) {
                try {
                    wait(maxWaitTimeMs - elapsedTimeMs, 0);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

}
