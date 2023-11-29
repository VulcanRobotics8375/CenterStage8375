package org.firstinspires.ftc.teamcode.robotcorelib.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;

public abstract class Subsystem {

    protected HardwareMap hardwareMap;

    protected Telemetry telemetry;

    protected Gamepad gamepad1;
    protected Gamepad gamepad2;

    public Subsystem() {
        Robot.getConfiguration().subsystems.add(this);
    }

    public abstract void init();

    public void stop() {}

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void setHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void assignGamepad1(Gamepad gamepad) {
        gamepad1 = gamepad;
    }

    public void assignGamepad2(Gamepad gamepad) {
        gamepad2 = gamepad;
    }

    public void assignGamePads(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

}
