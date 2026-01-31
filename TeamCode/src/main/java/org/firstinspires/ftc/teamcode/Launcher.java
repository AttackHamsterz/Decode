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
    private static final double CLOSE_ENOUGH_RPM_PERCENT = 8.0;
    private static final double CLOSE_ENOUGH_RPM = 200;
    private static final double MIN_LAUNCH_VALUE_RPM = 2350;
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
    private ArrayList<RPMEntry> velocityRPM;


    private class RPMEntry{
        public double value;
        public double rpm;
        public RPMEntry(double value, double rpm){
            this.value = value;
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

        velocityRPM = new ArrayList<>(List.of(
                new RPMEntry(-50,-1500),
                new RPMEntry(-40,-1200),
                new RPMEntry(-20,-500),
                new RPMEntry(0,0),
                new RPMEntry(20,500),
                new RPMEntry(40,1200),
                new RPMEntry(50,1500)
        ));


        // Table of distances in meters to RPM
        distanceRPM = new ArrayList<>(List.of(
                new RPMEntry(0.0, 0.0),
                new RPMEntry(1.0, 2520.0),
                new RPMEntry(1.07, 2620.0),
                new RPMEntry(1.25, 2790.0),
                new RPMEntry(1.45, 3010.0),
                new RPMEntry(1.72, 3240.0),
                new RPMEntry(2.0, 3360.0),
                new RPMEntry(2.4, 3620.0),
                new RPMEntry(2.43, 3670.0),
                new RPMEntry(2.63, 3860.0),
                new RPMEntry(3.28,4150),
                new RPMEntry(3.4, 4210.0),
                new RPMEntry(3.5,4400),
                new RPMEntry(3.8,4460),
                new RPMEntry(5.0, 5620.0)
        ));
        // added 20 to each on 11/23
        // added new far values 12/16
    }

    public void setRPMFromDistance(double distance, double extraRPM, double V){
        // Default RPM
        double rpm = 0;
        double vxrpm = 0;

        // Locate closest two points
        for(int i = 0; i < velocityRPM.size()-1; i++) {
            if (V < velocityRPM.get(0).value)
                vxrpm = velocityRPM.get(0).rpm;
            else if (V >= velocityRPM.get(velocityRPM.size()-1).value)
                vxrpm = velocityRPM.get(velocityRPM.size()-1).rpm;
            else if(velocityRPM.get(i).value < V && V < velocityRPM.get(i+1).value){
                double deltaX = velocityRPM.get(i+1).value - velocityRPM.get(i).value;
                double deltaRPM = velocityRPM.get(i+1).rpm - velocityRPM.get(i).rpm;
                vxrpm = velocityRPM.get(i).rpm + deltaRPM * ((V - velocityRPM.get(i).value) / deltaX);
            }
        }

        // Locate closest two points
        for(int i = 0; i < distanceRPM.size()-1; i++) {
            if (distance < distanceRPM.get(0).value)
                rpm = distanceRPM.get(0).rpm;
            else if (distance >= distanceRPM.get(distanceRPM.size()-1).value)
                rpm = distanceRPM.get(distanceRPM.size()-1).rpm;
            else if(distanceRPM.get(i).value < distance && distance < distanceRPM.get(i+1).value){
                double deltaDistance = distanceRPM.get(i+1).value - distanceRPM.get(i).value;
                double deltaRPM = distanceRPM.get(i+1).rpm - distanceRPM.get(i).rpm;
                rpm = distanceRPM.get(i).rpm + deltaRPM * ((distance - distanceRPM.get(i).value) / deltaDistance);
            }
        }
        setVelocityRPM(rpm+extraRPM+vxrpm);
    }

    //changing the distance based on the velocity. we need to figure out how fast we are going and in what direction, see the fiducial, and determine the rpm

    public void setVelocityRPM(double velocityRPM) {
        targetVelocityRPM = Range.clip(velocityRPM, MIN_RPM, MAX_RPM);
    }

    public boolean launchReady(){
        //double closeEnoughRPM = targetVelocityRPM * CLOSE_ENOUGH_RPM_PERCENT / 100.0;

        return targetVelocityRPM > MIN_LAUNCH_VALUE_RPM && Math.abs(deltaRPM) < CLOSE_ENOUGH_RPM;
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
