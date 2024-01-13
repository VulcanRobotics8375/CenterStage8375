package org.firstinspires.ftc.teamcode.robotcorelib.util;

public interface AutoTask extends Runnable {

    boolean conditional();

    void run();

}
