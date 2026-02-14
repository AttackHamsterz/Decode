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

    private static final double kP = 50.0;
    private static final double kI = 1.0;
    private static final double kD = 0.0;
    private static final double kF = 32767.0 / (MAX_RPM * RPM_TO_TPS);

    // Slew rate: max RPM change per loop tick (~100ms).
    // 1500 RPM/tick means 0→4000 RPM ramps over ~270ms (roughly 3 ticks).
    // This limits inrush current while keeping spin-up fast enough for gameplay.
    private static final double SLEW_UP_RPM_PER_TICK = 1500.0;

    private final DcMotorEx launchMotor;
    private volatile double targetVelocityRPM;  // What callers request
    private double interimVelocityRPM;           // Slewed value actually sent to motor
    private double currentVelocityRPM;

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
        launchMotor = ssom.hardwareMap.get(DcMotorEx.class, "launchMotor");

        // Setup PID and motor
        launchMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        launchMotor.setDirection(DcMotorEx.Direction.FORWARD);
        launchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initial Motor speeds
        targetVelocityRPM = 0;
        interimVelocityRPM = 0;
        currentVelocityRPM = 0;

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
                new RPMEntry(0.5, 2657.0),
                new RPMEntry(1.07, 2657.0),
                new RPMEntry(1.87, 3110.0),
                new RPMEntry(2.07, 3342.0),
                new RPMEntry(3.07, 4050.0),
                new RPMEntry(3.07, 4050.0),
                new RPMEntry(3.2, 4242.0),
                new RPMEntry(4.0, 4628.0),
                new RPMEntry(5.0, 4628.0)
        ));
    }

    public void setRPMFromDistance(double distance, double extraRPM, double V){
        double rpm = 0;
        double vxrpm = 0;

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

    public void setVelocityRPM(double velocityRPM) {
        targetVelocityRPM = Range.clip(velocityRPM, MIN_RPM, MAX_RPM);
    }

    public boolean launchReady(){
        // Check target (not slewed) RPM so we don't report ready mid-ramp
        return targetVelocityRPM > MIN_LAUNCH_VALUE_RPM && Math.abs(targetVelocityRPM - currentVelocityRPM) < CLOSE_ENOUGH_RPM;
    }

    @Override
    public void run() {
        setRunning();
        while (running) {
            // Slew interimVelocityRPM toward targetVelocityRPM
            // Ramp UP gradually to limit inrush current; ramp DOWN instantly
            if (interimVelocityRPM < targetVelocityRPM) {
                interimVelocityRPM = Math.min(targetVelocityRPM, interimVelocityRPM + SLEW_UP_RPM_PER_TICK);
            } else {
                interimVelocityRPM = targetVelocityRPM;
            }

            // Send slewed velocity to motor
            launchMotor.setVelocity(interimVelocityRPM * RPM_TO_TPS);
            currentVelocityRPM = launchMotor.getVelocity() * TPS_TO_RPM;

            // Don't saturate the thread
            sleep();
        }

        // Stop everything
        launchMotor.setVelocity(0);
        targetVelocityRPM = 0;
        interimVelocityRPM = 0;
        currentVelocityRPM = 0;
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
            telemetry.addData("launchInterimVelocity", interimVelocityRPM);
            telemetry.addData("launchCurrentVelocity", currentVelocityRPM);
        }
    }
}