package org.firstinspires.ftc.teamcode.robotcorelib.motion.kinematics;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public abstract class BasicMotionProfile {

    protected final double maxVel;
    protected final double maxAccel;
    protected double t;
    protected Pose2d start;
    protected Pose2d end;

    public BasicMotionProfile(double maxVel, double maxAccel, Pose2d start, Pose2d end) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.start = start;
        this.end = end;
    }

    public abstract BasicMotionProfile build();

    public abstract Pose2d getPosition(double t);
    public abstract Pose2d getVelocity(double t);
    public abstract Pose2d getAccel(double t);

    public double getTime() {
        return t;
    }

}
