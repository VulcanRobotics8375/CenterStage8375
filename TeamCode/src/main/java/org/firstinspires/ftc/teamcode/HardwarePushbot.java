/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HardwarePushbot
{
    /* Public OpMode members. */
    public DcMotor  leftFront   = null;
    public DcMotor  rightFront = null;
    public DcMotor  leftBack = null;
    public DcMotor  rightBack = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront  = hwMap.get(DcMotor.class, "LF");
        leftBack  = hwMap.get(DcMotor.class, "LB");
        rightFront  = hwMap.get(DcMotor.class, "RF");
        rightBack  = hwMap.get(DcMotor.class, "RB");

        // Reversing left motors
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power'

        setMotorPowers(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDrivetrainMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftBack.setMode(mode);
        rightBack.setMode(mode);
    }



    /**
     * Sets all motor powers to given speeds.
     * @param LFPower
     * @param RFPower
     * @param LBPower
     * @param RBPower
     */
   public void setMotorPowers(double LFPower, double RFPower, double LBPower, double RBPower) {
       leftFront.setPower(LFPower);
       rightFront.setPower(RFPower);
       leftBack.setPower(LBPower);
       rightBack.setPower(RBPower);
   }

    public void setMotorPowers(double allPower) {
        setMotorPowers(allPower, allPower, allPower, allPower);
    }

    public void moveByTime(double LFPower, double RFPower, double LBPower, double RBPower,
                           ElapsedTime runtime, double time,
                           LinearOpMode opMode, Telemetry telemetry) {
        setMotorPowers(LFPower, RFPower, LBPower, RBPower);
        while (opMode.opModeIsActive() && runtime.milliseconds() < time) {
            telemetry.addData("moveByTime running", "for " + time + " seconds");
            telemetry.addData("LFPower", LFPower);
            telemetry.addData("RFPower", RFPower);
            telemetry.addData("LBPower", LBPower);
            telemetry.addData("RBPower", RBPower);
            telemetry.update();
        }
        setMotorPowers(0);

    }
    public void moveByTime(double allPower, ElapsedTime runtime, double time, LinearOpMode opMode, Telemetry telemetry) {
        moveByTime(allPower, allPower, allPower, allPower, runtime, time, opMode, telemetry);
    }
 }
