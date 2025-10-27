package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher extends RobotPart<LauncherMetric>{
    public static final double IDLE_RPM = 2000.0;

    private static final double TPR = 28.0;
    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = 6000.0;
    private static final double CLOSE_ENOUGH_RPM = 150.0;
    private static final double TRIGGER_MAX_RPM = 6000.0;
    private static final double TPS_TO_RPM = 60.0 / TPR;
    private static final double RPM_TO_TPS = TPR / 60.0;

    private static final double kP = 50.0;  // Get to target quickly, little overshoot
    private static final double kI = 1.0;   // Doesn't reach target
    private static final double kD = 0.0;   // To reduce overshoot or oscillations
    private static final double kF = 32767.0 / (MAX_RPM * RPM_TO_TPS);   // Feed forward

    private final DcMotorEx launchMotor;
    private double targetVelocityRPM;
    private double currentVelocityRPM;
    private double deltaRPM;

    // RPM calculations
    private double currentRPM;

    public Launcher(StandardSetupOpMode ssom, boolean ignoreGamepad){
        this.ssom = ssom;
        setIgnoreGamepad(ignoreGamepad);
        launchMotor = ssom.hardwareMap.get(DcMotorEx.class, "launchMotor"); //need to define channel

        // Setup PID and motor
        launchMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        launchMotor.setDirection(DcMotorEx.Direction.FORWARD);
        launchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initial Motor speeds
        targetVelocityRPM = 0;
        currentVelocityRPM = 0;
        deltaRPM = 0;
    }

    public void setVelocityRPM(double velocityRPM) {
        targetVelocityRPM = Range.clip(velocityRPM, MIN_RPM, MAX_RPM);
    }

    public boolean launchReady(){
        return Math.abs(deltaRPM) < CLOSE_ENOUGH_RPM;
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            // Task a target velocity
            if (!ignoreGamepad)
                targetVelocityRPM = ssom.gamepadBuffer.g2LeftTrigger * TRIGGER_MAX_RPM;

            // Launch motor rpm
            launchMotor.setVelocity(targetVelocityRPM * RPM_TO_TPS);
            currentVelocityRPM = launchMotor.getVelocity() * TPS_TO_RPM;
            deltaRPM = targetVelocityRPM - currentVelocityRPM;

            // Don't saturate the thread
            sleep();
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
