package org.firstinspires.ftc.teamcode.robotcorelib.motion.followers;

import static org.firstinspires.ftc.teamcode.robotcorelib.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.robotcorelib.drive.DriveConstants.MAX_VEL;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.univariate.BrentOptimizer;
import org.apache.commons.math3.optim.univariate.SearchInterval;
import org.apache.commons.math3.optim.univariate.UnivariateObjectiveFunction;
import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.robotcorelib.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robotcorelib.math.control.PID;
import org.firstinspires.ftc.teamcode.robotcorelib.math.geometry.Angle;
import org.firstinspires.ftc.teamcode.robotcorelib.math.geometry.ParametricArcLength;
import org.firstinspires.ftc.teamcode.robotcorelib.math.geometry.ParametricDistance;
import org.firstinspires.ftc.teamcode.robotcorelib.math.control.SimplePID;
import org.firstinspires.ftc.teamcode.robotcorelib.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.kinematics.DriveKinematics;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.Path;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.PathPoint;

import java.util.ArrayList;

@Config
public class ParametricGuidingVectorField extends Follower {

    public volatile boolean following;
    public volatile boolean atEnd;
    public static double tP = -2, tI = -0.01, tD = 0.0;
    private SimplePID turnPID = new SimplePID(tP, tI, tD, -0.8, 0.8);
    public static double mtpP = 0.07, mtpI = 0.0001, mtpD = 0.0;
    SimplePID moveToPointPID = new SimplePID(mtpP, mtpI, mtpD, 0.0, 0.7);

    private OpMode opMode;

    SplineInterpolator splineInterpolator = new SplineInterpolator();
    LinearInterpolator linearInterpolator = new LinearInterpolator();

    BrentOptimizer optim = new BrentOptimizer(1e-10, 1e-14);
    PolynomialSplineFunction xSpline;
    PolynomialSplineFunction ySpline;
    UnivariateFunction headingFunction;
    ParametricDistance distance;
    ParametricArcLength arcLength;

    int numPoints = 0;

    private double CTEt = -1.0;

    private double robotDistanceTravelled = 0.0;

    public static double TANGENT_VECTOR_GAIN = 0.7;
    public static double CROSS_TRACK_ERROR_GAIN = 0.27;

    private double totalDistance;
    public double distanceFromEnd;

    private double speed;
    public static double minSpeed = 0.32;

    private boolean maintainHeading = false;

    private ElapsedTime timer = new ElapsedTime();

    public ParametricGuidingVectorField() {}

    public ParametricGuidingVectorField(LinearOpMode opMode) {
        this.opMode = opMode;
    }
    public ParametricGuidingVectorField(OpModePipeline opMode) {
        this.opMode = opMode;
    }

    public void followPath(Path path) {
        followPathAsync(path);
        while(!atEnd && !((LinearOpMode)(opMode)).isStopRequested()) {
            update();
        }
    }

    public void followPathAsync(Path path) {
        following = true;
        atEnd = false;
        robotDistanceTravelled = 0;
        CTEt = -1;
        speed = path.getStart().speed;
        maintainHeading = path.getMaintainHeading();


        ArrayList<PathPoint> guidePoints = path.asList();
        double[] t = new double[guidePoints.size()];
        double[] x = new double[guidePoints.size()];
        double[] y = new double[guidePoints.size()];
        double[] theta = new double[guidePoints.size()];
        for (int i = 0; i < guidePoints.size(); i++) {
            t[i] = (double)i / (guidePoints.size() - 1);
            PathPoint p = guidePoints.get(i);
            x[i] = p.x;
            y[i] = p.y;
            theta[i] = p.theta;
        }
        xSpline = splineInterpolator.interpolate(t, x);
        ySpline = splineInterpolator.interpolate(t, y);
        headingFunction = linearInterpolator.interpolate(t, theta);

        numPoints = guidePoints.size();

        distance = new ParametricDistance(xSpline, ySpline);
        arcLength = new ParametricArcLength(xSpline, ySpline);

        totalDistance = arcLength.value(1);
    }

    public void update() {
        Robot.update();

        Pose2d robotPose = Robot.getRobotPose();
        Pose2d robotVelocity = Robot.getRobotVelocity();

        if (following) {
            //robot arc length calculation
//            robotDistanceTravelled += Math.sqrt((robotVelocity.getX() * robotVelocity.getX()) + (robotVelocity.getY() * robotVelocity.getY()));

            distance.updatePos(new Vector(robotPose.getX(), robotPose.getY()));
            SearchInterval searchInterval;
            if (CTEt < 0) {
                searchInterval = new SearchInterval(0, 1);
            } else {
//            double low = optim.optimize(new MaxEval(500), new UnivariateObjectiveFunction(t -> Math.pow(arcLength.value(t) - 5, 2)), GoalType.MINIMIZE, new SearchInterval(0, CTEt)).getPoint();
//            double high = optim.optimize(new MaxEval(500), new UnivariateObjectiveFunction(t -> Math.pow(arcLength.value(t) - 10, 2)), GoalType.MINIMIZE, new SearchInterval(CTEt, 1)).getPoint();
                searchInterval = new SearchInterval(0, 1);
            }
            CTEt = optim.optimize(new MaxEval(1000), new UnivariateObjectiveFunction(distance), GoalType.MINIMIZE, searchInterval).getPoint();

            double xDerivative = xSpline.derivative().value(CTEt);
            double yDerivative = ySpline.derivative().value(CTEt);
            double derivativeHeading = FastMath.atan2(yDerivative, xDerivative);
            Vector tangentVec = new Vector(FastMath.cos(derivativeHeading), FastMath.sin(derivativeHeading)).multiply(TANGENT_VECTOR_GAIN);
            Vector crossTrackVec = new Vector(xSpline.value(CTEt) - robotPose.getX(), ySpline.value(CTEt) - robotPose.getY()).normalize().multiply(CROSS_TRACK_ERROR_GAIN);

            Vector resultant = tangentVec.plus(crossTrackVec);
//            double turnOutput = turnPID.run(Angle.diff(robotPose.getHeading(), headingFunction.value((double) ((int) (CTEt * (numPoints - 1) + 1)) / (numPoints - 1.0))));
            double turnOutput;
            if (!maintainHeading) {
                turnOutput = turnPID.run(Angle.diff(robotPose.getHeading(), headingFunction.value(CTEt)));
            } else {
                turnOutput = turnPID.run(Angle.diff(robotPose.getHeading(), derivativeHeading));
            }
//        double turnOutput = turnPid.getOutput(0, Angle.diff(robotPose.getHeading(), headingFunction.value((double)((int) (CTEt * (numPoints - 1) + 1)) / (numPoints - 1.0))));
//        double turnOutput = turnPid.run(Angle.diff(robotPose.getHeading(), 0));
//            double turnOutput = turnPid.run(MathUtils.calcAngularError(Math.PI/2, robotPose.getHeading()));
//        double translationalVectorScalar = 1.0 - (2.0 * Math.abs(turnOutput));
//        resultant = resultant.multiply(translationalVectorScalar);

            //Motion Profile Generation
            // convert continuous, time-variant motion profile to discrete, time-invariant motion profile
            //transfer function-- https://www.desmos.com/calculator/rlv4hdqutl
            double power = speed;
            double targetVel = MAX_VEL * power;
            double accelDistance = (targetVel*targetVel) / (2.0 * MAX_ACCEL);

            double m = (1 - minSpeed) / accelDistance;

            double originalSpeed = power;
            double distanceFromStart = arcLength.value(CTEt);
            distanceFromEnd = totalDistance - distanceFromStart;
            if(distanceFromEnd < accelDistance) {
                power *= m * distanceFromEnd;
            }
            else if(distanceFromStart < accelDistance) {
                double minSpeedStart = 0.5*power;
                power *= m * distanceFromStart;
                if(Math.abs(power) < minSpeedStart) {
                    power = minSpeedStart * Math.signum(originalSpeed);
                }
            }

            if(Math.abs(power) < minSpeed) {
                power = minSpeed * Math.signum(originalSpeed);
            }

            resultant = resultant.multiply(power / resultant.magnitude());
            if (maintainHeading) {
                turnOutput *= power;
            }

            double[] outputWheelVelocities = DriveKinematics.mecanumFieldVelocityToWheelVelocities(robotPose, new Pose2d(resultant.x, -resultant.y, turnOutput));
            Robot.drivetrain.setPowers(outputWheelVelocities);

            opMode.telemetry.addData("targetVelX", resultant.x);
            opMode.telemetry.addData("targetVelY", resultant.y);
            opMode.telemetry.addData("CTEt", CTEt);
            opMode.telemetry.addData("distance", distance.value(CTEt));
            opMode.telemetry.addData("target heading", headingFunction.value(CTEt));

            following = distanceFromEnd > 0.5;
            if (!following) {
                timer.reset();
            }
        } else if (!atEnd) {
            if (timer.milliseconds() > 1500) {
                atEnd = true;
                return;
            }
            Vector vel = new Vector(xSpline.value(1.0) - robotPose.getX(), robotPose.getY() - ySpline.value(1.0));
            double distance = vel.magnitude();
            vel = vel.divide(distance);
            double moveToPointOutput = moveToPointPID.run(distance) + 0.1;
            vel = vel.multiply(moveToPointOutput);
            double turnOutput;
            if (!maintainHeading) {
                turnOutput = turnPID.run(Angle.diff(robotPose.getHeading(), headingFunction.value(1.0)));
            } else {
                turnOutput = turnPID.run(Angle.diff(robotPose.getHeading(), FastMath.atan2(ySpline.derivative().value(1.0), xSpline.derivative().value(1.0))));
            }
            double[] motorPowers = DriveKinematics.mecanumFieldVelocityToWheelVelocities(robotPose, new Pose2d(vel.x, vel.y, turnOutput));
            Robot.drivetrain.setPowers(motorPowers);

            opMode.telemetry.addData("PID output", moveToPointOutput);
        }

        atEnd = robotPose.vec().distTo(new Vector2d(xSpline.value(1.0), ySpline.value(1.0))) < 0.1 &&
                Angle.diff(robotPose.getHeading(), headingFunction.value(1.0)) < 0.01 &&
                robotVelocity.getX() + robotVelocity.getY() + robotVelocity.getHeading() < 0.03;

        opMode.telemetry.addData("x", robotPose.getX());
        opMode.telemetry.addData("y", robotPose.getY());
        opMode.telemetry.addData("heading", robotPose.getHeading());
        opMode.telemetry.addData("distance from end", distanceFromEnd);
        opMode.telemetry.update();
    }

}
