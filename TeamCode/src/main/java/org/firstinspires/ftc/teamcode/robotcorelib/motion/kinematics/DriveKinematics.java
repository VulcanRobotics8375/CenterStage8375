package org.firstinspires.ftc.teamcode.robotcorelib.motion.kinematics;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.robotcorelib.drive.DrivetrainImpl;

/**
 * Kinematics class that translates velocity vector to wheel velocities for different drivebases.
 * the order in which wheel velocities are returned is defined in DrivetrainInterface.
 * @see DrivetrainImpl
 */
public class DriveKinematics {

    public static double[] tankVelocityToWheelVelocities(double forward, double turn) {
        double magnitude = Math.abs(forward) + Math.abs(turn);
        if(magnitude > 1.0) {
            forward *= 1.0 / magnitude;
            turn *= 1.0 / magnitude;
        }
        return new double[] {
                forward + turn,
                forward - turn,
                forward + turn,
                forward - turn
        };
    }

    public static double[] mecanumVelocityToWheelVelocities(Pose2d robotVelocity) {
        double multiplier = 2.0 / Math.sqrt(2.0);
        double theta = Math.atan2(robotVelocity.getX(), robotVelocity.getY()) - (Math.PI / 4.0);
        double xVel = robotVelocity.getX();
        double yVel = robotVelocity.getY();
        double headingVel = robotVelocity.getHeading();

        double magnitude = Math.abs(xVel) + Math.abs(yVel) + Math.abs(headingVel);
        if(magnitude > 1.0 ) {
            xVel *= 1.0 / magnitude;
            yVel *= 1.0 / magnitude;
            headingVel *= 1.0 / magnitude;
        }

        double speed = multiplier * Math.hypot(yVel, xVel);

        return new double[] {
                (speed * Math.cos(theta)) + headingVel,
                (speed * Math.sin(theta)) - headingVel,
                (speed * Math.sin(theta)) + headingVel,
                (speed * Math.cos(theta)) - headingVel
        };

    }

    /**
     * field centric mecanum
     *
     * @param robotFieldVelocity should be a normalized vector with components [x,y,theta]
     */
    public static double[] mecanumFieldVelocityToWheelVelocities(Pose2d robotFieldPose, Pose2d robotFieldVelocity) {
        double multiplier = 2.0 / Math.sqrt(2.0);
        double vx = robotFieldVelocity.getX();
        double vy = robotFieldVelocity.getY();
        double vHeading = robotFieldVelocity.getHeading();

        double magnitude = Math.abs(vx) + Math.abs(vy) + Math.abs(vHeading);
        if(magnitude > 1.0) {
            vx *= 1.0 / magnitude;
            vy *= 1.0 / magnitude;
            vHeading *= 1.0 / magnitude;
        }

        double speed = multiplier * Math.hypot(vx, vy);
        double theta = -robotFieldPose.getHeading() - (Math.atan2(vy, vx) - (Math.PI / 4.0));

        return new double[] {
                (speed * Math.cos(theta)) + vHeading,
                (speed * Math.sin(theta)) - vHeading,
                (speed * Math.sin(theta)) + vHeading,
                (speed * Math.cos(theta)) - vHeading
        };
    }

}
