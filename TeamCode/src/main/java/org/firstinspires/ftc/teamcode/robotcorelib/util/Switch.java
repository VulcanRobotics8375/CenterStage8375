package org.firstinspires.ftc.teamcode.robotcorelib.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Switch {
    boolean button;
    double lastPress = 0;
    ElapsedTime time = new ElapsedTime();
    public Switch(boolean button){ this.button = button; time.startTime();}

    public boolean simpleSwitch(boolean button) {
        if (button && !this.button) {
            this.button = true; return true;
        } else if (!button && this.button) {
            this.button = false; return false;
        }
        return false;
    }

    public boolean doubleClick(boolean button) {

        if (simpleSwitch(button)) {
            boolean plsreturn = time.milliseconds() <= 500;
            time.reset();
            return plsreturn;
        }

        return false;
    }


}
