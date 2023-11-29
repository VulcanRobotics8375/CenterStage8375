package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robotcorelib.math.control.PID;
import org.firstinspires.ftc.teamcode.robotcorelib.util.SubsystemState;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Switch;
import org.firstinspires.ftc.teamcode.robotcorelib.util.hardware.Encoder;


public class Lift extends SubsystemState {
    DcMotorEx lift;
    Encoder liftEncoder;

    PID liftPID = new PID(0.025,0.0008,0.01,0.1);
    private boolean liftHolding = false;
    private double minPos = 0;
    private final int LIFT_MAX_POS = 2800;
    private final double LIFT_CONVERGENCE_SPEED = 0.1;
    private final double LIFT_LIMIT_RANGE = 200.0;
    public double targetPos = 0.0;

    private int firstPixelPos = 715;
    private int pixelInc = 238;
    private Switch downSwitch = new Switch();
    private Switch upSwitch = new Switch();

    private LiftState liftState = LiftState.HOME;
    private LiftState prevLiftState = LiftState.HOME;

    private Switch aSwitch = new Switch();
    private Switch bSwitch = new Switch();
    private Switch xSwitch = new Switch();

    public void init() {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
//        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "front_right"));
        liftEncoder.setDirection(Encoder.Direction.REVERSE);

        liftPID.setOutputLimits(1.0);
    }

    public void intake() {
        home();
    }

    public void deposit() {
        double stick = -gamepad2.left_stick_y;
        boolean down = gamepad2.dpad_down;
        boolean up = gamepad2.dpad_up;

        int liftPos = getLiftPos();

        if(liftState == LiftState.HOME) {
            liftState = LiftState.INCREMENT;
        }

        if(Math.abs(stick) > 0.01) {
            liftState = LiftState.MANUAL;
        } else if(down || up) {
            liftState = LiftState.INCREMENT;
        }

        double liftPower = 0.0;

        switch (liftState) {
            case INCREMENT:
                if (prevLiftState != LiftState.INCREMENT) {
                    if (upSwitch.simpleSwitch(up)) {
                        targetPos = (1 + (liftPos - firstPixelPos) / pixelInc) * pixelInc + firstPixelPos;
                    } else if (downSwitch.simpleSwitch(down)) {
                        targetPos = ((liftPos - firstPixelPos) / pixelInc) * pixelInc + firstPixelPos;
                    }
                } else {
                    if (upSwitch.simpleSwitch(up)) {
                        targetPos += pixelInc;
                    } else if (downSwitch.simpleSwitch(down)) {
                        targetPos -= pixelInc;
                    }
                }

                targetPos = Range.clip(targetPos, firstPixelPos, firstPixelPos + 9*pixelInc);
                liftPower = liftPID.getOutput(liftPos, targetPos);

                break;

            case MANUAL:
                if (Math.abs(stick) > 0.01) {
                    if (liftHolding) {
                        liftHolding = false;
                        liftPID.reset();
                    }
                    if (stick > 0) {
                        if(liftPos > LIFT_MAX_POS - LIFT_LIMIT_RANGE) {
                            liftPower = stick - ((stick / LIFT_LIMIT_RANGE) * (liftPos - (LIFT_MAX_POS - LIFT_LIMIT_RANGE)));
                        } else {
                            liftPower = stick;
                        }
                    } else {
                        if(liftPos < minPos + LIFT_LIMIT_RANGE) {
                            liftPower = (stick / LIFT_LIMIT_RANGE) * (liftPos - minPos);
                        } else {
                            liftPower = stick;
                        }
                    }
                } else {
                    if (!liftHolding) {
                        targetPos = Range.clip(liftPos, minPos, LIFT_MAX_POS);
                        liftHolding = true;
                    }
                    liftPower = liftPID.getOutput(liftPos, targetPos);
                }

                break;
        }

        prevLiftState = liftState;

        lift.setPower(liftPower);

        telemetry.addData("lift pos", liftPos);
        telemetry.addData("lift power", liftPower);
        telemetry.addData("targetPos", targetPos);
        telemetry.addData("stick", stick);
        telemetry.addData("lift current", lift.getCurrent(CurrentUnit.AMPS));
    }

    public void transfer() {
        home();
    }

    public void home() {
        liftState = LiftState.HOME;
        prevLiftState = LiftState.HOME;

        targetPos = 0;
        double liftPower = liftPID.getOutput(getLiftPos(), targetPos);

        lift.setPower(liftPower);

        telemetry.addData("lift pos", getLiftPos());
        telemetry.addData("lift power", liftPower);
        telemetry.addData("lift current", lift.getCurrent(CurrentUnit.AMPS));
    }

    public void telemetryLift() {
        telemetry.addData("lift pos", getLiftPos());
    }

    public void test(double stick) {
        lift.setPower(stick*1);
        telemetryLift();
    }

    public int getLiftPos() {
        return liftEncoder.getCurrentPosition();
    }

    public boolean intakeReady() {
        return getLiftPos() < 15;
    }

    public void testPID(PID pid) {
        if (aSwitch.simpleSwitch(gamepad2.a)) {
            targetPos = 0;
        } else if (bSwitch.simpleSwitch(gamepad2.b)) {
            targetPos = 300;
        } else if (xSwitch.simpleSwitch(gamepad2.x)) {
            targetPos = 600;
        } else if (Math.abs(gamepad2.left_stick_y) > 0.01) {
            targetPos -= gamepad2.left_stick_y;
        }
        double liftPower = pid.getOutput(getLiftPos(), targetPos);
        lift.setPower(liftPower);

        telemetry.addData("lift pos", getLiftPos());
        telemetry.addData("target pos", targetPos);
        telemetry.addData("lift power", liftPower);
    }

    public enum LiftState {
        HOME,
        INCREMENT,
        MANUAL
    }
}
