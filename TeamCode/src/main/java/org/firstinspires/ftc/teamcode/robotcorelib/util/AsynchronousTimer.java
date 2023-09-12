package org.firstinspires.ftc.teamcode.robotcorelib.util;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AsynchronousTimer {
    public ElapsedTime timer;
    public AsynchronousTimer(ElapsedTime.Resolution resolution) {
        timer = new ElapsedTime(resolution);
        timer.reset();
    }

    public boolean elapsedTime(double timerLimit) {
        if (timer.time() >= timerLimit) {
            timer.reset();
            return true;
        }
        return false;
    }
}
