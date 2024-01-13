package org.firstinspires.ftc.teamcode.robotcorelib.util.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LVMaxbotixTest {

    public AnalogInput sensor;
    public static final double PRE_SCALAR_INCHES = 512.0;
    public DistanceUnit distanceUnit = DistanceUnit.INCH;

    public LVMaxbotixTest(HardwareMap hardwareMap, String deviceName) {
        sensor = hardwareMap.analogInput.get(deviceName);
    }

    public double getDistance(DistanceUnit distanceUnit) {
        double defaultOut = sensor.getVoltage() * (PRE_SCALAR_INCHES / sensor.getMaxVoltage());
        switch(distanceUnit) {
            case CM:
                return defaultOut * 2.54;
            case MM:
                return defaultOut * 25.4;
            case METER:
                return defaultOut * 0.254;
            default:
                return defaultOut;
        }
    }

    public double getDistance() {
        return getDistance(distanceUnit);
    }
}
