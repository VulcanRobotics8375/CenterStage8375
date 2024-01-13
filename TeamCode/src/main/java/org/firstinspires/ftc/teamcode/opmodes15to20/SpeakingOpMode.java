package org.firstinspires.ftc.teamcode.opmodes15to20;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SpeakingOpMode extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        telemetry.speak("I am Rock, destroyer of worlds. Cower before my powerful abilities.");
        telemetry.update();
    }

}
