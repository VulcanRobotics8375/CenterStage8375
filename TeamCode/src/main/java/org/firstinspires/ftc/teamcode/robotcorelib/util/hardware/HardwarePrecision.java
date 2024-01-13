package org.firstinspires.ftc.teamcode.robotcorelib.util.hardware;

public enum HardwarePrecision {

    LOW (10.0),
    MEDIUM (5.0),
    HIGH (1.0),
    ULTRA_HIGH(0.1);

    public double value;
    HardwarePrecision(double precision) {
        this.value = precision;
    }

}
