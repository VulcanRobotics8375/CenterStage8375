package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robotcorelib.math.control.SimplePID;
import org.firstinspires.ftc.teamcode.robotcorelib.math.filters.ExponentialMovingAverage;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;


public class Lift extends Subsystem {
    DcMotorEx lift;
    
    SimplePID liftPID = new SimplePID(0.005, 0.0, 0.0, -1.0, 1.0);
    private double liftTargetPos;
    private boolean liftHolding = false;
    private double minPos = 0;
    private final int LIFT_MAX_POS = 570;
    private final int LIFT_CLEARED_POS = 380;
    private final int LIFT_ALLIANCE_POS = 510;
    private final double LIFT_CONVERGENCE_SPEED = 0.1;
    private final double LIFT_LIMIT_RANGE = 100.0;

    public void init() {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
    }

    public void run(double stick) {
        double liftPos = lift.getCurrentPosition();
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
            liftPower = liftPID.run(liftTargetPos, liftPos) + feedForward;
        }

        telemetry.addData("lift PID feedforward", lift.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);
        telemetry.addData("lift power", liftPower);
        telemetry.addData("lift pos", liftPos);
        telemetry.addData("lift current", lift.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("lift holding", liftHolding);
        if(lift.getCurrent(CurrentUnit.AMPS) > 14) {
            liftPower *= 0.8;
        }
        lift.setPower(liftPower);
    }
}
