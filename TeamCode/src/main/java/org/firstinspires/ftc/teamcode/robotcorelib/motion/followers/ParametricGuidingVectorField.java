package org.firstinspires.ftc.teamcode.robotcorelib.motion.followers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

public class ParametricGuidingVectorField extends Follower {

    public volatile boolean following;
    private PID velocityPid = new PID(1.0, 0.0, 0.0);
    private PID turnPid = new PID(-1.5, -0.01, 0.0);

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

    private final double TANGENT_VECTOR_GAIN = 0.35;
    private final double CROSS_TRACK_ERROR_GAIN = 0.2;

    private double totalDistance;
    
    public ParametricGuidingVectorField() { PIDinit(); }

    public ParametricGuidingVectorField(LinearOpMode opMode) { 
        this.opMode = opMode;
        PIDinit();
    }
    public ParametricGuidingVectorField(OpModePipeline opMode) {
        this.opMode = opMode;
        PIDinit();
    }

    public void followPath(Path path) {
        followPathAsync(path);
        while(following && !((LinearOpMode)(opMode)).isStopRequested()) {
            update();
        }
    }

    public void followPathAsync(Path path) {
        following = true;
        robotDistanceTravelled = 0;
        CTEt = -1;

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
//        Robot.update();

        Pose2d robotPose = Robot.getRobotPose();

        //robot arc length calculation
        Pose2d robotVelocity = Robot.getRobotVelocity();
        robotDistanceTravelled += Math.sqrt((robotVelocity.getX() * robotVelocity.getX()) + (robotVelocity.getY()* robotVelocity.getY()));

        distance.updatePos(new Vector(robotPose.getX(), robotPose.getY()));
        SearchInterval searchInterval;
        if(CTEt < 0) {
            searchInterval = new SearchInterval(0, 1);
        } else {
            double low = optim.optimize(new MaxEval(500), new UnivariateObjectiveFunction(t -> arcLength.value(t) - 5), GoalType.MINIMIZE, new SearchInterval(0, CTEt)).getPoint();
            double high = optim.optimize(new MaxEval(500), new UnivariateObjectiveFunction(t -> arcLength.value(t) - 10), GoalType.MINIMIZE, new SearchInterval(CTEt, 1)).getPoint();
            searchInterval = new SearchInterval(low, high);
        }
        CTEt = optim.optimize(new MaxEval(1000), new UnivariateObjectiveFunction(distance), GoalType.MINIMIZE, searchInterval).getPoint();

        double xDerivative = xSpline.derivative().value(CTEt);
        double yDerivative = ySpline.derivative().value(CTEt);
        double derivativeHeading = FastMath.atan2(yDerivative, xDerivative);
        Vector tangentVec = new Vector(FastMath.cos(derivativeHeading), FastMath.sin(derivativeHeading)).multiply(TANGENT_VECTOR_GAIN);
        Vector crossTrackVec = new Vector(xSpline.value(CTEt) - robotPose.getX(), ySpline.value(CTEt) - robotPose.getY()).multiply(CROSS_TRACK_ERROR_GAIN);

        Vector resultant = tangentVec.plus(crossTrackVec);
        double vectorAngle = tangentVec.angle();
        double turnOutput = turnPid.getOutput(0, Angle.diff(robotPose.getHeading(), headingFunction.value((double)((int) (CTEt * (numPoints - 1) + 1)) / (numPoints - 1.0))));
//        double turnOutput = turnPid.run(Angle.diff(robotPose.getHeading(), 0));
//            double turnOutput = turnPid.run(MathUtils.calcAngularError(Math.PI/2, robotPose.getHeading()));
        double translationalVectorScalar = 1.0 - (2.0 * Math.abs(turnOutput));
//        resultant = resultant.multiply(translationalVectorScalar);
        double targetSpeed = (totalDistance - arcLength.value(CTEt) < 2) ? 0.2 : 0.9;
        double power = velocityPid.getOutput(Robot.getRobotVelocity().vec().norm(), targetSpeed);
        resultant = resultant.multiply(power / resultant.magnitude());

        double[] outputWheelVelocities = DriveKinematics.mecanumFieldVelocityToWheelVelocities(robotPose, new Pose2d(resultant.x, -resultant.y, turnOutput));
        Robot.drivetrain.setPowers(outputWheelVelocities);

        following = robotPose.vec().distTo(new Vector2d(xSpline.value(1), ySpline.value(1))) > 0.5;

        opMode.telemetry.addData("x", robotPose.getX());
        opMode.telemetry.addData("y", robotPose.getY());
        opMode.telemetry.addData("heading", robotPose.getHeading());
        opMode.telemetry.addData("targetVelX", resultant.x);
        opMode.telemetry.addData("targetVelY", resultant.y);
    }

    private void PIDinit() {
        velocityPid.setOutputLimits(1);
        turnPid.setOutputLimits(0.8);
        velocityPid.setOutputRampRate(0.05);
        turnPid.setOutputRampRate(0.05);
    }

}
