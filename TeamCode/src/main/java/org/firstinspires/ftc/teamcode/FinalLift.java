package org.firstinspires.ftc.teamcode;

import android.net.EthernetNetworkSpecifier;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FinalLift extends RobotPart<FinalLiftMetric>{
    public FinalLift(StandardSetupOpMode ssom){
        this.ssom = ssom;
        this.gamepad = ssom.gamepad1;
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


