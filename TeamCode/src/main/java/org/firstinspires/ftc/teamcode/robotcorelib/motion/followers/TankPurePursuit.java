package org.firstinspires.ftc.teamcode.robotcorelib.motion.followers;

import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.Path;

@Deprecated
public class TankPurePursuit extends Follower {

    Path path;

    public TankPurePursuit(Path path) {
        super();
        this.path = path;
    }

    protected void run() {
//        Pose2d robotPose2d = Robot.getRobotPose();
//        Point robotPose = new Point(robotPose2d.getX(), robotPose2d.getY());
//
//        ArrayList<Point> circleIntersections;
//
//        Point followPoint = new Point();
//        int followPointIndex = 0;
//
//        for (int i = 0; i < path.size() - 1; i++) {
//            Point start = path.getPoint(i);
//            Point end = path.getPoint(i + 1);
//
//            if(path.getSpeed(i) < 0) {
//                robotPose2d = new Pose2d(robotPose.x, robotPose.y, MathUtils.fullAngleWrap(robotPose2d.getHeading() + Math.PI));
//            }
//
//            if(i+1 == path.size()) {
//                if(end.x < start.x) {
//                    circleIntersections = MathUtils.lineCircleIntersect(start, end, path.getLookahead(i+1), robotPose, false, true);
//                } else {
//                    circleIntersections = MathUtils.lineCircleIntersect(start, end, path.getLookahead(i+1), robotPose, true, false);
//                }
//            } else {
//                circleIntersections = MathUtils.lineCircleIntersect(start, end, path.getLookahead(i+1), robotPose, true, false);
//            }
//
//            double closestAngle = Double.MAX_VALUE;
//            for(Point intersection : circleIntersections) {
//                double angle = Math.atan2(intersection.y - robotPose.y, intersection.x - robotPose.x);
//                double relativeAngleToPoint = Math.abs(MathUtils.angleWrap(angle - robotPose2d.getHeading()));
//
//                if(Math.abs(relativeAngleToPoint) < closestAngle) {
//                    followPoint.setPoint(intersection);
//                    followPointIndex = i+1;
//                }
//            }
//
//            if(i+1 == path.size()) {
//                double maxX = Math.max(start.x, end.x);
//                double minX = Math.min(start.x, end.x);
//                if(followPoint.x > maxX || followPoint.x < minX) {
//                    followPoint.setPoint(end);
//                    followPointIndex = i+1;
//                }
//            }
//        }
//
//        double absoluteAngleToPoint = MathUtils.fullAngleWrap(Math.atan2(followPoint.y - robotPose.y, followPoint.x - robotPose.x));
//        double robotAngle = Math.abs(absoluteAngleToPoint - robotPose2d.getHeading()) > Math.PI ? absoluteAngleToPoint - Math.signum(absoluteAngleToPoint - robotPose2d.getHeading()) * 2.0 * Math.PI : robotPose2d.getHeading();
//
//        //TODO add PID controller to this @matt
//        double error = absoluteAngleToPoint - robotAngle;
//        double turnSpeed = error * path.getTurnSpeed(followPointIndex);
//        double speed = path.getSpeed(followPointIndex);


//        Robot.drivetrain.setPowers(powers);

    }

}
