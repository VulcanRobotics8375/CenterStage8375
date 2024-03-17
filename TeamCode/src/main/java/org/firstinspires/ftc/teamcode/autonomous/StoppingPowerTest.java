package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotcorelib.math.control.PID;
import org.firstinspires.ftc.teamcode.robotcorelib.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.AutoPipeline;

public class StoppingPowerTest extends AutoPipeline {
//    Δx = (Vf)/(V0+2a)
//    wtf does this mean??
    Vector pos = new Vector();

    PID pidx,pidy;

    Vector posEstimate;
    Vector posIdeal;
    Vector velIdeal;
    Vector delta = new Vector();
    Vector acc;
    Vector idealChange;
    Vector initialVelocity;
    Vector finalVelocity;
    ElapsedTime timer = new ElapsedTime();
    double lastTime;
    double dT;

    Vector motorPowers;

    public StoppingPowerTest(Vector pos){
        timer.startTime();
        this.pos = pos;
    }

    public void update(Vector pos, Vector posIdeal) {
        dT = lastTime - timer.milliseconds();
        initialVelocity = finalVelocity;
        finalVelocity = new Vector((this.pos.x - pos.x)/dT, (this.pos.y - pos.y)/dT);
        acc = new Vector((finalVelocity.x - initialVelocity.x)/dT, (finalVelocity.y - initialVelocity.y)/dT);
        this.pos = pos;
        lastTime = timer.milliseconds();
        idealChange = new Vector(posIdeal.x-pos.x, posIdeal.y-pos.y);
        posEstimate.x = finalVelocity.x/(initialVelocity.x+(2*acc.x));
        posEstimate.y = finalVelocity.y/(initialVelocity.y+(2*acc.y));
    }

    public Vector stop(Vector motorPowers){
        double idealX = idealChange.x;
        double idealY = idealChange.y;
        if (Math.abs(idealX)<=Math.abs(posEstimate.x)+1.0 || Math.abs(idealY)<=Math.abs(posEstimate.y)+1.0) {return motorPowers;}
        return new Vector(pidx.getOutput(finalVelocity.x,idealX),pidy.getOutput(finalVelocity.y,idealY));
    }


}
