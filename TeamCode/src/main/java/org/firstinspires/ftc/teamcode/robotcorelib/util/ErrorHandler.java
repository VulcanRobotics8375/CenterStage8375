package org.firstinspires.ftc.teamcode.robotcorelib.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class ErrorHandler {
    private Telemetry telemetry;
    private ArrayList<String> messages = new ArrayList<>();

    public ErrorHandler(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void addMessage(String message) {
        messages.add(message);
    }

    public void update(boolean updateTelemetry) {
        for(String message : messages) {
            telemetry.addLine("ERR: " + message);
        }
        if(updateTelemetry) {
            telemetry.update();
        }
    }

    public void clear() {
        messages.clear();
    }


}
