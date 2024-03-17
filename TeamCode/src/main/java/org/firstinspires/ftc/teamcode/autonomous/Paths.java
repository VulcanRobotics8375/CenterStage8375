package org.firstinspires.ftc.teamcode.autonomous;


import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.Path;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.PathBuilder;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Pose2d;

public class Paths {
    private double speed;
    private Pose2d start = new Pose2d();

    public Paths(double speed) {
        this.speed = speed;
    }

    public Path getSpikeMarkPath(boolean red, boolean backDrop, int propIdx) {
        boolean trussOnLeft = (red && backDrop) || !(red || backDrop);
        if (trussOnLeft && propIdx == 0) {
            return new PathBuilder()
                .speed(speed)
                .turnSpeed(0.5)
                .start(start)
                .addGuidePoint(new Pose2d(13.3, 0, 0))
                .end(new Pose2d(23.8, 5.8, 0.6665))
                .build();
        } else if (propIdx == 1) {
            return new PathBuilder()
                .speed(speed)
                .turnSpeed(0.5)
                .start(start)
                .addGuidePoint(new Pose2d(10, 0.1, 0))
                .end(new Pose2d(27.5, 0, 0))
                .build();
        } else if (trussOnLeft && propIdx == 2) {
            return new PathBuilder()
                .speed(speed)
                .turnSpeed(0.5)
                .maintainHeading(true)
                .start(start)
                .addGuidePoint(new Pose2d(9.25, -3.5, 0))
                .end(new Pose2d(18.5, -6.767, 0))
                .build();
        } else if (!trussOnLeft && propIdx == 0) {
            return new PathBuilder()
                .speed(speed)
                .turnSpeed(0.5)
                .maintainHeading(true)
                .start(start)
                .addGuidePoint(new Pose2d(9.25, 4.0, 0))
                .end(new Pose2d(19.5, 7.767, 0))
                .build();
        } else if (!trussOnLeft && propIdx == 2) {
            return new PathBuilder()
                .speed(speed)
                .turnSpeed(0.5)
                .start(start)
                .addGuidePoint(new Pose2d(13.3, 0, 0))
                .end(new Pose2d(23.8, -4.8, -0.6665))
                .build();
        } else {
            return new PathBuilder()
                .speed(speed)
                .turnSpeed(0.5)
                .start(start)
                .addGuidePoint(new Pose2d(10, 0.1, 0))
                .end(new Pose2d(27.5, 0, 0))
                .build();
        }
    }

    public Path getBackDropPath(Pose2d robotPose, boolean red, boolean backDrop, int propIdx) {
        double backDropPos;
        if (red && backDrop) {
            backDropPos = 27.008 - 6 * (propIdx - 1);
            return new PathBuilder()
                    .speed(0.5)
                    .turnSpeed(0.5)
                    .start(robotPose)
                    .addGuidePoint(new Pose2d(12.57, -2.49))
                    .addGuidePoint(new Pose2d(7.62, -12.558, 0.803))
                    .addGuidePoint(new Pose2d(8.2, -24.9, Math.PI/2))
                    .addGuidePoint(new Pose2d(backDropPos, -31, Math.PI/2))
                    .end(new Pose2d(backDropPos, -40.5, Math.PI/2))
                    .build();
        } else if (!red && backDrop) {
            backDropPos = 27.008 + 6 * (propIdx - 1);
            return new PathBuilder()
                    .speed(0.5)
                    .turnSpeed(0.5)
                    .start(robotPose)
                    .addGuidePoint(new Pose2d(12.57, 2.49, 0.0))
                    .addGuidePoint(new Pose2d(7.62, 12.558, -0.803))
                    .addGuidePoint(new Pose2d(8.2, 24.9, -Math.PI/2))
                    .addGuidePoint(new Pose2d(backDropPos, 31, -Math.PI/2))
                    .end(new Pose2d(backDropPos, 39.461, -Math.PI/2))
                    .build();
        } else if (red && !backDrop) {
            backDropPos = 27.41 - 6 * (propIdx - 1);
            return new PathBuilder()
                    .speed(0.5)
                    .turnSpeed(0.5)
                    .start(robotPose)
                    .addGuidePoint(new Pose2d(3.51, 0.8366, Math.PI))
                    .addGuidePoint(new Pose2d(3.556, -45.19, Math.PI))
                    .addGuidePoint(new Pose2d(12.5339, -57.782, 1.878))
                    .addGuidePoint(new Pose2d(18.104, -74.722, Math.PI/2))
                    .addGuidePoint(new Pose2d(backDropPos, -79.59, Math.PI/2))
                    .end(new Pose2d(backDropPos, -87.009, Math.PI/2))
                    .build();
        } else { // !red && !backDrop
            backDropPos = 27.41 + 6 * (propIdx - 1);
            return new PathBuilder()
                    .speed(0.5)
                    .turnSpeed(0.5)
                    .start(robotPose)
                    .addGuidePoint(new Pose2d(3.51, -0.8366, -Math.PI))
                    .addGuidePoint(new Pose2d(3.556, 45.19, -Math.PI))
                    .addGuidePoint(new Pose2d(12.5339, 57.782, -1.878))
                    .addGuidePoint(new Pose2d(18.104, 74.722, -Math.PI/2))
                    .addGuidePoint(new Pose2d(backDropPos, 79.59, -Math.PI/2))
                    .end(new Pose2d(backDropPos, 87.009, -Math.PI/2))
                    .build();
        }
    }

    public Path getParkPath(boolean red, boolean parkLeft, int propIdx) {
        Robot.setRobotPose(new Pose2d(0, 6 * (propIdx - 1), 0));
        double yPark;
        if (parkLeft && red) {
            yPark = -27.4;
        } else if (!parkLeft && red) {
            yPark = 26;
        } else if (parkLeft && !red) {
            yPark = -26;
        } else {
            yPark = 27.4;
        }
        return new PathBuilder()
                .start(new Pose2d())
                .addGuidePoint(new Pose2d(5, 0, 0))
                .addGuidePoint(new Pose2d(4.135, yPark, 0))
                .end(new Pose2d(-9.06, yPark, 0))
                .build();
    }
}
