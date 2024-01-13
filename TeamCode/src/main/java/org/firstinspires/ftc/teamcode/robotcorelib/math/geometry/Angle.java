package org.firstinspires.ftc.teamcode.robotcorelib.math.geometry;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

public class Angle {

    public static double normalize(double angle) {
        while(angle < 0) {
            angle += 2.0 * PI;
        }
        while(angle >= 2.0 * PI) {
            angle -= 2.0 * PI;
        }
        return angle;
    }

    public static double diff(double angle1, double angle2) {
        double theta2Corrected = abs(angle2 - angle1) > PI ? angle2 - signum(angle2 - angle1) * 2.0 * PI : angle2;

        return theta2Corrected - angle1;
    }

}
