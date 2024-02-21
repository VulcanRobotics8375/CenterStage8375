package org.firstinspires.ftc.teamcode.robotcorelib.util;

public class Toggle {
    private boolean button = false;
    private boolean toggle = false;

    public boolean toggle(boolean button) {
        if (button && !this.button) {
            this.button = true;
            toggle = true;
            return true;
        } else if (!button && this.button) {
            this.button = false;
        }
        toggle = false;
        return false;
    }

    public boolean getToggle() {
        return toggle;
    }
}
