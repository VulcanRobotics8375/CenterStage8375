package org.firstinspires.ftc.teamcode.robotcorelib.util.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.configuration.annotations.AnalogSensorType;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;

@AnalogSensorType
@DeviceProperties(name = "analog encoder", xmlTag = "analog_encoder")
public class AnalogEncoder extends AnalogInput {

    private double pos;
    private double lastPos;
    private Mode mode = Mode.ABSOLUTE; // default mode is absolute
    private int totalRotations = 0;
    public static final double WRAPAROUND_THRESHOLD = 1.0; //wraparound threshold in volts

    /**
     * Constructor
     *
     * @param controller AnalogInput controller this channel is attached to
     * @param channel    channel on the analog input controller
     */
    public AnalogEncoder(AnalogInputController controller, int channel) {
        super(controller, channel);
    }

    public void update() {
        lastPos = pos;
        pos = getVoltage() / getMaxVoltage();
        if(pos - lastPos > WRAPAROUND_THRESHOLD / getMaxVoltage()) {
            totalRotations--;
        }
        if(pos - lastPos < -1.0 * WRAPAROUND_THRESHOLD / getMaxVoltage()) {
            totalRotations++;
        }
    }

    public double getCurrentPosition() {
        if(mode == Mode.ABSOLUTE) {
            return pos;
        } else {
            return totalRotations + pos;
        }
    }

    public double getCurrentPosition(Mode mode) {
        if(mode == Mode.ABSOLUTE) {
            return pos;
        } else {
            return totalRotations + pos;
        }
    }



    public double getCurrentVelocity() {
        return pos - lastPos;
    }

    public enum Mode {
        ABSOLUTE,
        INCREMENTAL
    }

}
