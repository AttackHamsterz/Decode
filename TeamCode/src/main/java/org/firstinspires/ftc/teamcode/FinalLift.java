package org.firstinspires.ftc.teamcode;

import android.net.EthernetNetworkSpecifier;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FinalLift extends RobotPart<FinalLiftMetric>{

    public final DcMotorEx leftLift, rightLift;

    public FinalLift(StandardSetupOpMode ssom){
        this.ssom = ssom;
        this.gamepad = ssom.gamepad1;
        leftLift = ssom.hardwareMap.get(DcMotorEx.class, "leftLiftDrive"); //need to define channel
        rightLift = ssom.hardwareMap.get(DcMotorEx.class, "rightLiftDrive"); //need to define channel

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void run() {
        if (!ignoreGamepad) {
            while (!isInterrupted()) {
                if (gamepad.start) {
                    // code to lift robot
                }
            }
        }

    }

    @Override
    protected void setTo(FinalLiftMetric metric) {

    }

    @Override
    public void safeHold() {

    }

    @Override
    protected boolean closeEnough(FinalLiftMetric metric) {
        return false;
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {

    }
}


