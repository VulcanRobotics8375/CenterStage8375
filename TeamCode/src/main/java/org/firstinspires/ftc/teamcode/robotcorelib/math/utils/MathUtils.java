package org.firstinspires.ftc.teamcode.robotcorelib.math.utils;

import static java.lang.Math.PI;
import static java.lang.Math.pow;

import org.firstinspires.ftc.teamcode.robotcorelib.util.JoystickCurve;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Point;
import org.firstinspires.ftc.teamcode.robotcorelib.util.hardware.HardwarePrecision;

import java.util.ArrayList;

public class MathUtils {

    /**
     * Constrains angle to range [-pi,pi]
     * @param angle input angle in radians
     * @return constrained output
     */
    public static double angleWrap(double angle) {
        while(angle <= -PI) {
            angle += 2.0 * PI;
        }
        while(angle > PI) {
            angle -= 2.0 * PI;
        }
        return angle;
    }

    /**
     * Constrains angle to range [0, 2pi]
     * @param angle input angle in radians
     * @return constrained output
     */
    public static double fullAngleWrap(double angle) {
        while(angle <= 0) {
            angle += 2.0 * PI;
        }
        while(angle > 2.0 * PI) {
            angle -= 2.0 * PI;
        }
        return angle;
    }

    /**
     * BOTH ANGLES SHOULD BE IN RANGE [0,2pi]
     */
    public static double calcAngularError(double expected, double measurement) {
        double correctedExpectedValue = Math.abs(expected - measurement) > PI ? expected - (Math.signum(expected - measurement) * 2.0 * PI) : expected;
        return correctedExpectedValue - measurement;
    }

    /**
     *
     * @param in joystick input
     * @param curveType curve type based on what drivers want
     * @return scaled joystick input. All inputs should be in the range [-1,1] to scale correctly.
     */
    public static double joystickCurve(double in, JoystickCurve curveType) {
        switch (curveType) {
            case CUBIC:
                return pow(in, 3);
            case MODIFIED_CUBIC:
                return (in / 1.07) * (0.62 * pow(in, 2) + 0.45);
            default:
                return in;
        }
    }

    /**
     *
     * @param linePoint1 point 1 on lerp line
     * @param linePoint2 point 2 on lerp line
     * @param radius radius of the circle
     * @param circleCenter origin of the circle
     * @param minBox toggle min boundary of line
     * @param maxBox toggle max boundary of line
     * @return the intersection points of a line segment and a circle.
     */
    public static ArrayList<Point> lineCircleIntersect(Point linePoint1, Point linePoint2, double radius, Point circleCenter, boolean minBox, boolean maxBox) {
        ArrayList<Point> intersections = new ArrayList<Point>();

        double m = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);
        double b = linePoint1.y - (m * linePoint1.x);

        double quadraticA = pow(m, 2) + 1;

        double quadraticB = 2.0 * ((m * b) - (m * circleCenter.y) - circleCenter.x);

        double quadraticC = pow(circleCenter.y, 2) - pow(radius, 2) + pow(circleCenter.x, 2) - (2.0 * b * circleCenter.y) + pow(b, 2);

        try {

            if(linePoint2.x - linePoint1.x == 0) {
                double x1 = linePoint2.x;
                double y1 = Math.sqrt((-1.0 * Math.pow(x1, 2)) + (2.0 * x1 * circleCenter.x) - Math.pow(circleCenter.y, 2) + Math.pow(radius, 2)) + circleCenter.y;

                double minY = Math.min(linePoint1.y, linePoint2.y);
                double maxY = Math.max(linePoint1.y, linePoint2.y);

                if (minBox && maxBox) {
                    if (y1 > minY && y1 < maxY) {
                        intersections.add(new Point(x1, y1));
                    }
                } else if (maxBox) {
                    if (y1 < maxY) {
                        intersections.add(new Point(x1, y1));
                    }
                } else if (minBox) {
                    if (y1 > minY) {
                        intersections.add(new Point(x1, y1));
                    }
                }

                double y2 =  (-1.0 * Math.sqrt((-1.0 * Math.pow(x1, 2)) + (2.0 * x1 * circleCenter.x) - Math.pow(circleCenter.y, 2) + Math.pow(radius, 2))) + circleCenter.y;

                if (minBox && maxBox) {
                    if (y1 > minY && y1 < maxY) {
                        intersections.add(new Point(x1, y2));
                    }
                } else if (maxBox) {
                    if (y1 < maxY) {
                        intersections.add(new Point(x1, y2));
                    }
                } else if (minBox) {
                    if (y1 > minY) {
                        intersections.add(new Point(x1, y2));
                    }
                }
            }
            else if(linePoint2.y == linePoint1.y) {
                double x1 = circleCenter.x + Math.sqrt((-1.0 * Math.pow(linePoint2.y, 2)) + (2.0 * linePoint2.y * circleCenter.y) - Math.pow(circleCenter.y, 2) + radius);
                double y1 = linePoint2.y;

                double minX = Math.min(linePoint1.x, linePoint2.x);
                double maxX = Math.max(linePoint1.x, linePoint2.x);

                if (minBox && maxBox) {
                    if (x1 >= minX && x1 <= maxX) {
                        intersections.add(new Point(x1, y1));
                    }
                } else if (maxBox) {
                    if (x1 <= maxX) {
                        intersections.add(new Point(x1, y1));
                    }
                } else if (minBox) {
                    if (x1 >= minX) {
                        intersections.add(new Point(x1, y1));
                    }
                }

                double x2 = circleCenter.x - Math.sqrt((-1.0 * Math.pow(linePoint2.y, 2)) + (2.0 * linePoint2.y * circleCenter.y) - Math.pow(circleCenter.y, 2) + radius);

                if (minBox && maxBox) {
                    if (x2 > minX && x2 < maxX) {
                        intersections.add(new Point(x2, y1));
                    }
                } else if (maxBox) {
                    if (x2 < maxX) {
                        intersections.add(new Point(x2, y1));
                    }
                } else if (minBox) {
                    if (x2 >= minX) {
                        intersections.add(new Point(x2, y1));
                    }
                }
            }
            else {
                double x1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
                double y1 = (m * x1) + b;

                double minX = Math.min(linePoint1.x, linePoint2.x);
                double maxX = Math.max(linePoint1.x, linePoint2.x);

                if (minBox && maxBox) {
                    if (x1 > minX && x1 < maxX) {
                        intersections.add(new Point(x1, y1));
                    }
                } else if (maxBox) {
                    if (x1 < maxX) {
                        intersections.add(new Point(x1, y1));
                    }
                } else if (minBox) {
                    if (x1 >= minX) {
                        intersections.add(new Point(x1, y1));
                    }
                }

                double x2 = (-quadraticB - Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
                double y2 = (m * x2) + b;

                if (minBox && maxBox) {
                    if (x2 > minX && x2 < maxX) {
                        intersections.add(new Point(x2, y2));
                    }
                } else if (maxBox) {
                    if (x2 < maxX) {
                        intersections.add(new Point(x2, y2));
                    }
                } else if (minBox) {
                    if (x2 >= minX) {
                        intersections.add(new Point(x2, y2));
                    }
                }
            }
        } catch (Exception ignored) {}

        return intersections;

    }

    public static boolean shouldHardwareUpdate(double input, double currentValue, HardwarePrecision precision) {
        return Math.abs((input / currentValue) - 1) > (precision.value / 100.0);
    }

}

