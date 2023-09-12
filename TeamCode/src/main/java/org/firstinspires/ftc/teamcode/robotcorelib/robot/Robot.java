package org.firstinspires.ftc.teamcode.robotcorelib.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotcorelib.drive.DrivetrainImpl;
import org.firstinspires.ftc.teamcode.robotcorelib.drive.localization.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.AutoPipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.util.ErrorHandler;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

import java.util.List;

public class Robot {

    private static HardwareMap hardwareMap;
    private static Telemetry telemetry;
    private static ErrorHandler errorHandler;

    private static StandardTrackingWheelLocalizer localizer;

    private static Pose2d robotPose = new Pose2d();
    private static Pose2d robotVelocity = new Pose2d();

    private static boolean running;

    private static List<LynxModule> hubs;
    //default is manual since robotcorelib is set up to do so
    private static LynxModule.BulkCachingMode bulkCachingMode = LynxModule.BulkCachingMode.MANUAL;

    private static RobotConfig config;

    public static RobotRunMode runMode;
    public static DrivetrainImpl drivetrain;

    public static void init(OpModePipeline opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        config = opMode.subsystems;
        config.init();
        config.localizer = new StandardTrackingWheelLocalizer(opMode.hardwareMap);
        localizer = config.localizer;

        errorHandler = new ErrorHandler(telemetry);
        hubs = opMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(bulkCachingMode);
        }
        boolean drivetrainExists = false;
        for (Subsystem sub : config.subsystems) {
            sub.setHardwareMap(opMode.hardwareMap);
            sub.setTelemetry(opMode.telemetry);
            sub.assignGamePads(opMode.gamepad1, opMode.gamepad2);
            if(sub instanceof DrivetrainImpl) {
                drivetrain = (DrivetrainImpl) sub; // cast drivetrain to drivetrainImpl, this is for backend controller stuff.
                drivetrainExists = true;
            }
            if(!drivetrainExists) {
                errorHandler.addMessage("ERR: No Drivetrain Found in RobotConfig");
            }
            sub.init();
        }

    }

    public static <subClass> Subsystem getSubsystemFromConfig(Class<? extends Subsystem> subClass) {
        for (Subsystem subsystem : config.subsystems) {
            if(subsystem.getClass().isInstance(subClass)) {

            }
        }
        return null;
    }

    public static void init(AutoPipeline opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        config = opMode.subsystems;
        config.init();
        config.localizer = new StandardTrackingWheelLocalizer(opMode.hardwareMap);
        localizer = config.localizer;

        errorHandler = new ErrorHandler(telemetry);
        hubs = opMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(bulkCachingMode);
        }
        boolean drivetrainExists = false;
        for (Subsystem sub : config.subsystems) {
            sub.setHardwareMap(opMode.hardwareMap);
            sub.setTelemetry(opMode.telemetry);
            sub.assignGamePads(opMode.gamepad1, opMode.gamepad2);
            if(sub instanceof DrivetrainImpl) {
                drivetrain = (DrivetrainImpl) sub; // cast drivetrain to drivetrainImpl, this is for backend controller stuff.
                drivetrainExists = true;
            }
            if(!drivetrainExists) {
                errorHandler.addMessage("ERR: No Drivetrain Found in RobotConfig");
            }
            sub.init();
        }

    }

    public static Pose2d getRobotPose() {
        return robotPose;
    }

    public static void setRobotPose(Pose2d robotPose) {
        Robot.localizer.setPose(robotPose);
        Robot.robotPose = robotPose;
    }

    public static Pose2d getRobotVelocity() {
        return robotVelocity;
    }

    private static void setRobotVelocity(Pose2d robotVelocity) {
        Robot.robotVelocity = robotVelocity;
    }

    public static RobotConfig getConfiguration() {
        return config;
    }

    public static HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public static void setBulkCachingMode(LynxModule.BulkCachingMode bulkCachingMode) {
        Robot.bulkCachingMode = bulkCachingMode;
    }

    public static LynxModule.BulkCachingMode getBulkCachingMode() {
        return bulkCachingMode;
    }

    public static void updateGlobalPosition() {
        localizer.update();
        robotPose = localizer.getPose();
        robotVelocity = localizer.getVelocity();
    }

    public static void clearBulkCache() {
        if(bulkCachingMode == LynxModule.BulkCachingMode.MANUAL) {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }
        }
    }

    public static void addErrorMessage(String message) {
        errorHandler.addMessage(message);
    }

    public static void update() {
        clearBulkCache();
        updateGlobalPosition();
//        errorHandler.update(false);
//        telemetry.update();
    }

    public static void stop() {
        for (Subsystem sub : config.subsystems) {
            sub.stop();
        }
    }

    public static void setRunMode(RobotRunMode runMode) {
        Robot.runMode = runMode;
    }

}
