package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robotcorelib.math.control.SimplePID;
import org.firstinspires.ftc.teamcode.robotcorelib.math.filters.ExponentialMovingAverage;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;
import org.firstinspires.ftc.teamcode.robotcorelib.util.hardware.Encoder;


public class Lift extends Subsystem {
    DcMotorEx lift;
    Encoder liftEncoder;

    SimplePID liftPID = new SimplePID(0.007, 0.0, 0.0, -1.0, 1.0);
    private double liftTargetPos;
    private boolean liftHolding = false;
    private double minPos = 0;
    private final int LIFT_MAX_POS = 2510;
    private final double LIFT_CONVERGENCE_SPEED = 0.1;
    private final double LIFT_LIMIT_RANGE = 200.0;
    private double targetPos = 0.0;

    public void init() {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "front_left"));
    }

    public void run(double stick) {
        int liftPos = getLiftPos();

//        targetPos = Range.clip(targetPos - stick*0.1, minPos, LIFT_MAX_POS);
        double liftPower;
        if (Math.abs(stick) > 0.05) {
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
                liftTargetPos = Range.clip(liftPos, minPos, LIFT_MAX_POS);
                liftHolding = true;
            }
            double feedForward = 0.03;
            liftPower = liftPID.run(liftTargetPos, liftPos);
        }

        lift.setPower(liftPower);
        telemetry.addData("lift pos", liftPos);
        telemetry.addData("targetPos", targetPos);
        telemetry.addData("stick", stick);
        telemetry.addData("lift current", lift.getCurrent(CurrentUnit.AMPS));
//        if(lift.getCurrent(CurrentUnit.AMPS) > 14) {
//            liftPower *= 0.8;
//        }
//        lift.setPower(-liftPower);
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
}
