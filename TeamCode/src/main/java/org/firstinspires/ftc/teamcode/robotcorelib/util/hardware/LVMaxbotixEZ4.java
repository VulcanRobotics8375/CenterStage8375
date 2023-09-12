package org.firstinspires.ftc.teamcode.robotcorelib.util.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.configuration.annotations.AnalogSensorType;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@AnalogSensorType
@DeviceProperties(name = "LVMaxbotixEZ4", xmlTag = "LVMaxbotixEZ4")
public class LVMaxbotixEZ4 extends AnalogInput {


    public double PRE_SCALAR_INCHES = 12.0 / 0.5;
    public double VOLTAGE_OFFSET = 0.0;
    public DistanceUnit distanceUnit = DistanceUnit.INCH;

    /**
     * Constructor
     *
     * @param controller AnalogInput controller this channel is attached to
     * @param channel    channel on the analog input controller
     */
    public LVMaxbotixEZ4(AnalogInputController controller, int channel) {
        super(controller, channel);
    }

    public double getDistance(DistanceUnit distanceUnit) {
        double defaultOut = (getVoltage() - VOLTAGE_OFFSET) * (PRE_SCALAR_INCHES);
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

    public double getRawVoltage() {
        return getVoltage();
    }

    public double getDistance() {
        return getDistance(distanceUnit);
    }

}
