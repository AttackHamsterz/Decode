package org.firstinspires.ftc.teamcode;

import java.util.List;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher extends RobotPart<LauncherMetric>{
    public static final double IDLE_RPM = 2000.0;

    private static final double TPR = 28.0;
    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = 6000.0;
    private static final double CLOSE_ENOUGH_RPM = 150.0;
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

    private ArrayList<RPMEntry> distanceRPM;

    private class RPMEntry{
        public double distance;
        public double rpm;
        public RPMEntry(double distance, double rpm){
            this.distance = distance;
            this.rpm = rpm;
        }
    };

    // RPM calculations
    private double currentRPM;

    public Launcher(StandardSetupOpMode ssom){
        this.ssom = ssom;
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

        // Table of distances in meters to RPM
        distanceRPM = new ArrayList<>(List.of(
                new RPMEntry(0.0, 0.0),
                new RPMEntry(1.0, 2520.0),
                new RPMEntry(1.45, 2870.0),
                new RPMEntry(1.8, 3120.0),
                new RPMEntry(2.4, 3620.0),
                new RPMEntry(3.3, 4260.0),
                new RPMEntry(4.0, 4620.0),
                new RPMEntry(5.0, 5620.0)
        ));
        // added 20 to each on 11/23
    }

    public void setRPMFromDistance(double distance, double extraRPM){
        // Default RPM
        double rpm = 0;

        // Locate closest two points
        for(int i = 0; i < distanceRPM.size()-1; i++){
            if(distanceRPM.get(i).distance < distance && distance < distanceRPM.get(i+1).distance){
                double deltaDistance = distanceRPM.get(i+1).distance - distanceRPM.get(i).distance;
                double deltaRPM = distanceRPM.get(i+1).rpm - distanceRPM.get(i).rpm;
                rpm = distanceRPM.get(i).rpm + deltaRPM * ((distance - distanceRPM.get(i).distance) / deltaDistance);
            }
        }
        setVelocityRPM(rpm+extraRPM);
    }

    public void setVelocityRPM(double velocityRPM) {
        targetVelocityRPM = Range.clip(velocityRPM, MIN_RPM, MAX_RPM);
    }

    public boolean launchReady(){
        return targetVelocityRPM > 0 && Math.abs(deltaRPM) < CLOSE_ENOUGH_RPM;
    }

    @Override
    public void run() {
        setRunning();
        while (running) {
            // Task a target velocity
            //if (!ssom.gamepadBuffer.ignoreGamepad)
            //    targetVelocityRPM = ssom.gamepadBuffer.g2LeftTrigger * TRIGGER_MAX_RPM;

            // Launch motor rpm
            launchMotor.setVelocity(targetVelocityRPM * RPM_TO_TPS);
            currentVelocityRPM = launchMotor.getVelocity() * TPS_TO_RPM;
            deltaRPM = targetVelocityRPM - currentVelocityRPM;

            // Don't saturate the thread
            sleep();
        }

        // Stop everything
        launchMotor.setVelocity(0);
        targetVelocityRPM = 0;
        currentVelocityRPM = 0;
        deltaRPM = 0;
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
        if((DEBUG & 2) != 0) {
            telemetry.addData("launchTargetVelocity", targetVelocityRPM);
            telemetry.addData("launchCurrentVelocity", currentVelocityRPM);
        }
    }
}
