package org.firstinspires.ftc.teamcode.robotcorelib.util;

public class Toggle {
    private boolean button = false;

    public boolean toggle(boolean button) {
        if (button && !this.button) {
            this.button = true;
            return true;
        } else if (!button && this.button) {
            this.button = false;
        }
        return false;
    }
}
