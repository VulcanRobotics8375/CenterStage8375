package org.firstinspires.ftc.teamcode.robotcorelib.motion.kinematics;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class TrapezoidalMotionProfile extends BasicMotionProfile {

    //TODO all of this lmfao idk what i am doing
    //TODO get rid of pose2d, should just be scalars for distance for basic shit

    public TrapezoidalMotionProfile(double maxVel, double maxAccel, Pose2d start, Pose2d end) {
        super(maxVel, maxAccel, start, end);
    }

    /**
     * all equations in this method are derived from this desmos graph:
     * https://www.desmos.com/calculator/wfnk0qbd87
     */
    @Override
    public TrapezoidalMotionProfile build() {
        double distance = Math.hypot(end.getX() - start.getX(), end.getY() - start.getY());
        t = (maxVel / maxAccel) + (distance / maxVel);
        return this;
    }

    @Override
    public Pose2d getPosition(double t) {
        if (t < maxVel / maxAccel) {
            return null;
        }
        return null;
    }

    @Override
    public Pose2d getVelocity(double t) {
        return null;
    }

    @Override
    public Pose2d getAccel(double t) {
        return null;
    }
}
