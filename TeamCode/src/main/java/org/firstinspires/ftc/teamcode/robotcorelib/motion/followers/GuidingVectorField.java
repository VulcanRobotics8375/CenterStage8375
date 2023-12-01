package org.firstinspires.ftc.teamcode.robotcorelib.motion.followers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.univariate.BrentOptimizer;
import org.apache.commons.math3.optim.univariate.SearchInterval;
import org.apache.commons.math3.optim.univariate.UnivariateObjectiveFunction;
import org.apache.commons.math3.optim.univariate.UnivariatePointValuePair;
import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.robotcorelib.math.geometry.Angle;
import org.firstinspires.ftc.teamcode.robotcorelib.math.geometry.Distance;
import org.firstinspires.ftc.teamcode.robotcorelib.math.control.SimplePID;
import org.firstinspires.ftc.teamcode.robotcorelib.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.kinematics.DriveKinematics;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.Path;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.PathPoint;

import java.util.ArrayList;

public class GuidingVectorField extends Follower {

    public volatile boolean following;
    private SimplePID velocityPid = new SimplePID(1.0, 0.0, 0.0, -1.0, 1.0);
    private SimplePID turnPid = new SimplePID(-1.5, -0.01, 0.0, -0.8, 0.8);

    private OpMode opMode;

    SplineInterpolator interpolator = new SplineInterpolator();
    BrentOptimizer optim = new BrentOptimizer(1e-10, 1e-14);
    PolynomialSplineFunction spline;
    Distance distance;

    private final double TANGENT_VECTOR_GAIN = 0.3;
    private final double CROSS_TRACK_ERROR_GAIN = 0.3;

    public GuidingVectorField() {}

    public GuidingVectorField(LinearOpMode opMode) { this.opMode = opMode; }
    public GuidingVectorField(OpModePipeline opMode) {
        this.opMode = opMode;
    }

    public void followPath(Path path) {
        followPathAsync(path);
        while(following && !((LinearOpMode)(opMode)).isStopRequested()) {
            update();
        }
    }

    public void followPathAsync(Path path) {
        following = true;

        ArrayList<PathPoint> guidePoints = path.asList();
        double[] x = new double[guidePoints.size()];
        double[] y = new double[guidePoints.size()];
        for (int i = 0; i < guidePoints.size(); i++) {
            PathPoint p = guidePoints.get(i);
            x[i] = p.x;
            y[i] = p.y;
        }
        spline = interpolator.interpolate(x, y);
        distance = new Distance(spline);
    }

    public void update() {
        Robot.update();

        Pose2d robotPose = Robot.getRobotPose();

        //robot arc length calculation
        Pose2d robotVelocity = Robot.getRobotVelocity();

        distance.updatePos(new Vector(robotPose.getX(), robotPose.getY()));
        double currentX = robotPose.getX();
        UnivariatePointValuePair minValue = optim.optimize(new MaxEval(1000), new UnivariateObjectiveFunction(distance), GoalType.MINIMIZE, new SearchInterval(currentX, spline.getKnots()[spline.getN()]));

        double derivative = spline.derivative().value(minValue.getPoint());
        double derivativeHeading = FastMath.atan2(derivative, 1);
        Vector tangentVec = new Vector(FastMath.cos(derivativeHeading), FastMath.sin(derivativeHeading)).multiply(TANGENT_VECTOR_GAIN);
        Vector crossTrackVec = new Vector(minValue.getPoint() - robotPose.getX(), spline.value(minValue.getPoint()) - robotPose.getY()).multiply(CROSS_TRACK_ERROR_GAIN);

        Vector resultant = tangentVec.plus(crossTrackVec);
        double vectorAngle = tangentVec.angle();
        double turnOutput = turnPid.run(Angle.diff(robotPose.getHeading(), vectorAngle));
//            double turnOutput = turnPid.run(MathUtils.calcAngularError(Math.PI/2, robotPose.getHeading()));
        double translationalVectorScalar = 1.0 - (2.0 * Math.abs(turnOutput));
        resultant = resultant.multiply(translationalVectorScalar);

        double[] outputWheelVelocities = DriveKinematics.mecanumFieldVelocityToWheelVelocities(robotPose, new Pose2d(resultant.x, -resultant.y, turnOutput));
        Robot.drivetrain.setPowers(outputWheelVelocities);

        following = robotPose.vec().distTo(new Vector2d(spline.getKnots()[spline.getN()], spline.value(spline.getKnots()[spline.getN()]))) > 0.5;

        opMode.telemetry.addData("x", robotPose.getX());
        opMode.telemetry.addData("y", robotPose.getY());
        opMode.telemetry.addData("heading", robotPose.getHeading());
        opMode.telemetry.addData("targetVelX", resultant.x);
        opMode.telemetry.addData("targetVelY", resultant.y);
    }

}
