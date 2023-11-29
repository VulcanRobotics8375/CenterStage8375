package org.firstinspires.ftc.teamcode.robotcorelib.drive.localization;


import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotcorelib.util.hardware.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */

public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer implements Localizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.688975;// in
    public static double LEFT_MULTIPLIER = 1.0225820196;
    public static double RIGHT_MULTIPLIER = 1.00990677691;
    public static double STRAFE_MULTIPLIER = 1.03248010325;
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    //Original Wheelbase: 8.34655886296
    public static double LATERAL_DISTANCE_OFFSET = 8.48074079761; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -6.038218407; // in; offset of the lateral wheel

    private double lastRightPos;

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, (LATERAL_DISTANCE_OFFSET / 2.0), 0), // left
                new Pose2d(0, -(LATERAL_DISTANCE_OFFSET / 2.0), 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(-90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "back_left"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "back_right"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "front_left"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        rightEncoder.setDirection(Encoder.Direction.FORWARD);
        leftEncoder.setDirection(Encoder.Direction.FORWARD);
        frontEncoder.setDirection(Encoder.Direction.FORWARD);
        lastRightPos = rightEncoder.getCurrentPosition();

    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double encoderTicksToInches(double ticks, double wheelRadius) {
        return wheelRadius * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * LEFT_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * RIGHT_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * STRAFE_MULTIPLIER
        );

    }

    public List<Integer> getRawWheelPositions() {
        return Arrays.asList(
          leftEncoder.getCurrentPosition(),
          rightEncoder.getCurrentPosition(),
          frontEncoder.getCurrentPosition()
        );

    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity())
        );
    }

    @Override
    public Pose2d getPose() {
        return getPoseEstimate();
    }

    @Override
    public Pose2d getVelocity() {
        return getPoseVelocity();
    }

    @Override
    public void setPose(Pose2d pose) {
        setPoseEstimate(pose);
    }
}
