package org.firstinspires.ftc.teamcode.mechanisms1to10;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ProgrammingBoard2 {
    private DigitalChannel touchSensor;

    public void init(HardwareMap hwMap) {
        touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
    }
    public boolean getTouchSensorState() {
        return !touchSensor.getState();
    }
    public boolean isTouchSensorReleased() {
        return !getTouchSensorState();
    }
}
