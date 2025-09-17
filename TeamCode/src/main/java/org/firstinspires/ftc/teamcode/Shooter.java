package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter extends RobotPart<ShooterMetric>{
    public Shooter(StandardSetupOpMode ssom){
        this.ssom = ssom;
        this.gamepad = ssom.gamepad1;
    }

    @Override
    public void run() {

    }

    @Override
    protected void setTo(ShooterMetric metric) {

    }

    @Override
    public void safeHold() {

    }

    @Override
    
    
    protected boolean closeEnough(ShooterMetric metric) {
        return false;
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {

    }
}
