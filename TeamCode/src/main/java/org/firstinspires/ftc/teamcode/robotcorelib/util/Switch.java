package org.firstinspires.ftc.teamcode.robotcorelib.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Switch {
    private boolean button = false;

    public boolean simpleSwitch(boolean button) {
        if (button && !this.button) {
            this.button = true;
            return true;
        } else if (!button && this.button) {
            this.button = false;
        }
        return false;
    }
}
