package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher extends RobotPart<LauncherMetric>{
    private final DcMotor launchMotor;
    private double launchMotorPower;

    public Launcher(StandardSetupOpMode ssom){
        this.ssom = ssom;
        launchMotor = ssom.hardwareMap.get(DcMotor.class, "launchMotor"); //need to define channel
        // Setup
        launchMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setTargetPosition(0);
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotorPower = 0;
    }

    @Override
    public void run() {
        if (!ignoreGamepad) {
            while (!isInterrupted()) {
                launchMotor.setPower(ssom.gamepadBuffer.g2LeftTrigger);
                launchMotorPower = ssom.gamepadBuffer.g2LeftTrigger;

                sleep();
            }
        }
    }

    @Override
    protected void setTo(LauncherMetric metric) {

    }

    @Override
    public void safeHold() {

    }

    @Override
    
    
    protected boolean closeEnough(LauncherMetric metric) {
        return false;
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        telemetry.addData("launchMotorPower", launchMotorPower);
    }
}
