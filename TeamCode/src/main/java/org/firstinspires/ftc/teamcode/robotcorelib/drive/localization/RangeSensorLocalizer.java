package org.firstinspires.ftc.teamcode.robotcorelib.drive.localization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robotcorelib.util.hardware.LVMaxbotixEZ4;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Range Sensor localization using 2 walls.
 *
 * Mode Enum Reference:
 * --------- x ----------
 * ----------------------
 * |        north       |  |
 * |          |         |  |
 * |          |         |  |
 * |   west -- -- east  |  y
 * |          |         |  |
 * |          |         |  |
 * |        south       |  |
 * ----------------------
 *        AUDIENCE
 *
 * for freight frenzy, the north-west corner would be the blue alliance warehouse.
 * the order (NORTH_WEST vs WEST_NORTH) refers to what set of sensors is facing towards the wall.
 * The sensors facing forward (or backward) on the wall come first, and the perpendicular(normal)
 * sensors are the second direction. ( FORWARD_NORMAL )
 *
 */
public class RangeSensorLocalizer implements Localizer {
    public static double ANGLE_THRESHOLD = Math.toRadians(15);

    public DistanceUnit distanceUnit = DistanceUnit.INCH;
    private Pose2d poseEstimate = new Pose2d();
    public Mode mode = Mode.NORTH_WEST;
    public String configurationValidator;
    private List<Pose2d> sensorPoses;

    private LVMaxbotixEZ4 forward_1, forward_2, normal_1, normal_2;
    private double relativeOffsetForward, relativeOffsetNormal, robotOffsetForward, robotOffsetNormal, distanceForward, distanceNormal;

    @Override
    public void update() {

    }

    @Override
    public Pose2d getPose() {
        return null;
    }

    @Override
    public Pose2d getVelocity() {
        return null;
    }

    @Override
    public void setPose(Pose2d pose) {

    }


    enum Mode {
        NORTH_WEST, //forward sensors on north wall, normal sensors on west wall
        NORTH_EAST,
        SOUTH_WEST,
        SOUTH_EAST,
        EAST_NORTH,
        EAST_SOUTH,
        WEST_NORTH,
        WEST_SOUTH
    }

    static class ConfigurationValidator {
        public boolean pass;
        public String message;

        public ConfigurationValidator(boolean pass, String message) {
            this.pass = pass;
            this.message = message;
        }

        @Override
        public String toString() {
            if(pass) {
                return "Configuration Valid";
            } else {
                return "Localizer Configuration Error: " + message;
            }
        }

    }

    public RangeSensorLocalizer(HardwareMap hardwareMap) {
        // sensor positions relative to the center of the robot.
        // This is different from dead-wheel odo because it is not the center of rotation.
        sensorPoses = Arrays.asList(
                new Pose2d(0, 0, 0),
                new Pose2d(0, 0, 0),
                new Pose2d(0, 0, Math.toRadians(90)),
                new Pose2d(0, 0, Math.toRadians(90))
        );
        Collections.sort(sensorPoses, (o1, o2) -> {
            if (o1.getHeading() < o2.getHeading()) {
                return -1;
            } else if(o1.getHeading() > o2.getHeading()) {
                return 1;
            } else {
                return 0;
            }
        });

        configurationValidator = validateConfiguration(sensorPoses).toString();

        robotOffsetForward = (sensorPoses.get(0).getX() + sensorPoses.get(1).getX()) / 2.0;
        robotOffsetNormal = (sensorPoses.get(2).getY() + sensorPoses.get(3).getY()) /2.0;
        relativeOffsetForward = (sensorPoses.get(0).getX() - sensorPoses.get(1).getX()) / 2.0;
        relativeOffsetNormal = (sensorPoses.get(2).getY() - sensorPoses.get(3).getY()) / 2.0;
        distanceForward = Math.hypot(sensorPoses.get(1).getX() - sensorPoses.get(0).getX(), sensorPoses.get(1).getY() - sensorPoses.get(0).getY());
        distanceNormal = Math.hypot(sensorPoses.get(3).getX() - sensorPoses.get(2).getX(), sensorPoses.get(3).getY() - sensorPoses.get(2).getY());

        forward_1 = (LVMaxbotixEZ4) hardwareMap.get(AnalogInput.class, "forward_1");
        forward_2 = (LVMaxbotixEZ4) hardwareMap.analogInput.get("forward_2");
        normal_1 = (LVMaxbotixEZ4) hardwareMap.analogInput.get("normal_1");
        normal_2 = (LVMaxbotixEZ4) hardwareMap.analogInput.get("normal_2");

    }

    private Pose2d estimatePose() {
        List<Double> distances = getDistances();

        Pose2d poseEstimate = new Pose2d();
        double thetaEstimate;
        double xEstimate = 0;
        double yEstimate = 0;

        //forward distances
        double d1 = distances.get(0) - relativeOffsetForward;
        double d2 = distances.get(1) + relativeOffsetForward;

        double forwardTheta = Math.atan((d1 - d2) / distanceForward);
        double forwardDistance = ((d1 + d2) / 2.0) * Math.cos(forwardTheta);

        //normal distances
        double d3 = distances.get(2) - relativeOffsetNormal;
        double d4 = distances.get(3) + relativeOffsetNormal;

        double normalTheta = Math.atan((d3 - d4) / distanceNormal);
        double normalDistance = ((d3 + d4) / 2.0) * Math.cos(normalTheta);

        thetaEstimate = (forwardTheta + normalTheta) / 2.0;

        switch(mode) {
            case NORTH_WEST:
                xEstimate = normalDistance - 72.0;
                yEstimate = 72.0 - forwardDistance;
                break;
            case NORTH_EAST:
                xEstimate = 72.0 - normalDistance;
                yEstimate = 72.0 - forwardDistance;
                break;
            case SOUTH_WEST:
                xEstimate = normalDistance - 72.0;
                yEstimate = forwardDistance - 72.0;
                break;
            case SOUTH_EAST:
                xEstimate = 72.0 - normalDistance;
                yEstimate = forwardDistance - 72.0;
                break;
            case EAST_NORTH:
                xEstimate = 72.0 - forwardDistance;
                yEstimate = 72.0 - normalDistance;
                break;
            case EAST_SOUTH:
                xEstimate = 72.0 - forwardDistance;
                yEstimate = normalDistance - 72.0;
                break;
            case WEST_NORTH:
                xEstimate = forwardDistance - 72.0;
                yEstimate = 72.0 - normalDistance;
                break;
            case WEST_SOUTH:
                xEstimate = forwardDistance - 72.0;
                yEstimate = normalDistance - 72.0;
                break;
        }

        return new Pose2d(xEstimate, yEstimate, thetaEstimate);
    }

    private ConfigurationValidator validateConfiguration(List<Pose2d> sensorPositions) {
        if(sensorPositions.size() != 4) {
            return new ConfigurationValidator(false, "localizer requires 4 sensors");
        }
        else {
            int forwardNum = 0;
            int normalNum = 0;
            for (int i = 0; i < 4; i++) {
                Vector2d headingVec = sensorPositions.get(i).headingVec();
                if(Math.abs(headingVec.getX()) == 1) {
                    forwardNum++;
                } else if(Math.abs(headingVec.getY()) == 1) {
                    normalNum++;
                }
            }
            if(forwardNum != 2 || normalNum != 2) {
                return new ConfigurationValidator(false, "Cannot solve localization with this configuration");
            }
        }
        return new ConfigurationValidator(true, "config passed");
    }

    public List<Double> getDistances() {
        return Arrays.asList(
                forward_1.getDistance(distanceUnit),
                forward_2.getDistance(distanceUnit),
                normal_1.getDistance(distanceUnit),
                normal_2.getDistance(distanceUnit)
        );
    }

    public void setDistanceUnit(DistanceUnit unit) {
        this.distanceUnit = unit;
    }

}
