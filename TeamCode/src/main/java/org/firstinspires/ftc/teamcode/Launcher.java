package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher extends RobotPart<LauncherMetric>{
    private static final double TPR = 28.0;
    private static final double IDLE_RPM = 1000.0;
    private static final double MAX_RPM = 6000.0;
    private static final double TRIGGER_MAX_RPM = 5000.0;
    private static final double TPS_TO_RPM = 60.0 / TPR;
    private static final double RPM_TO_TPS = TPR / 60.0;

    private static final double kP = 50.0;  // Get to target quickly, little overshoot
    private static final double kI = 1.0;   // Doesn't reach target
    private static final double kD = 0.0;   // To reduce overshoot or oscillations
    private static final double kF = 32767.0 / (MAX_RPM * RPM_TO_TPS);   // Feed forward

    private final DcMotorEx launchMotor;
    private double targetVelocityRPM;
    private double currentVelocityRPM;

    // RPM calculations
    private double currentRPM;

    public Launcher(StandardSetupOpMode ssom){
        this.ssom = ssom;
        launchMotor = ssom.hardwareMap.get(DcMotorEx.class, "launchMotor"); //need to define channel
        // Setup
        launchMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        launchMotor.setDirection(DcMotorEx.Direction.FORWARD);
        launchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void run() {
        if (!ignoreGamepad) {
            while (!isInterrupted()) {
                // Task a target velocity
                targetVelocityRPM = ssom.gamepadBuffer.g2LeftTrigger * TRIGGER_MAX_RPM;
                launchMotor.setVelocity(targetVelocityRPM * RPM_TO_TPS);

                // Launch motor rpm
                currentVelocityRPM = launchMotor.getVelocity() * TPS_TO_RPM;

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
        telemetry.addData("launchTargetVelocity", targetVelocityRPM);
        telemetry.addData("launchCurrentVelocity", currentVelocityRPM);
    }
}
