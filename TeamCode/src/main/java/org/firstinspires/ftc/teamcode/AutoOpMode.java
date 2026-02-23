package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.content.SharedPreferences;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;
import java.util.List;

/**
 * This super auto entry point runs everything that all auto's will need
 */
@Autonomous(name = "Auto", group = "Robot")
@Disabled
public class AutoOpMode extends StandardSetupOpMode {
    protected final double MAX_PICKUP_VOLTAGE = 12.9;
    protected final double MIN_PICKUP_POWER = 0.30;
    protected final double MIN_PICKUP_VOLTAGE = 12.75;
    protected final double MAX_PICKUP_POWER = 0.30;
    protected final double PATH_VELOCITY_PERCENTAGE = 1.0;

    protected double pickupPower = 0.2;
    protected Timer pathTimer;

    // Finite state machine state
    protected List<Integer> launchPattern = new ArrayList<>();
    protected List<Sorter.BallColor> colorPattern = new ArrayList<>();
    protected int launchIndex = 0;
    protected int pathState;

    /* There are two major types of paths components: BezierCurves and BezierLines.
     *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
     *    - Control points manipulate the curve between the start and end points.
     *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
     *    * BezierLines are straight, and require 2 points. There are the start and end points.
     * Paths have can have heading interpolation: Constant, Linear, or Tangential
     *    * Linear heading interpolation:
     *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
     *    * Constant Heading Interpolation:
     *    - Pedro will maintain one heading throughout the entire path.
     *    * Tangential Heading Interpolation:
     *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
     * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
     * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */
    public void buildPaths() {}

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {}

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * Helper method to increment the path state by one
     */
    public void incrementPathState(){
        setPathState(pathState+1);
    }

    @Override public void init() {
        super.init();
        pathTimer = new Timer();
        buildPaths();

        // Start the eye to look for the obelisk
        eye.start();
    }


    @Override public void start() {
        // Set the starting path state
        setPathState(0);
        opmodeTimer.resetTimer();

        //Setup launch pattern
        int id = eye.getFiducialID();

        // When looking at the front of the robot, green should be right, purple back and left
        if (id == 21) {

            //GPP
            colorOrder = Eye.ColorOrder.GPP;

            //Counter Clockwise to Green, Clockwise to Purple1, Clockwise to Purple2
            launchPattern.add(-1);
            launchPattern.add(1);
            launchPattern.add(1);
            //
            launchPattern.add(-1);
            launchPattern.add(1);
            launchPattern.add(1);
            //
            launchPattern.add(-1);
            launchPattern.add(1);
            launchPattern.add(1);
            //
            colorPattern.add(Sorter.BallColor.Green);
            colorPattern.add(Sorter.BallColor.Purple);
            colorPattern.add(Sorter.BallColor.Purple);
            colorPattern.add(Sorter.BallColor.Green);
            colorPattern.add(Sorter.BallColor.Purple);
            colorPattern.add(Sorter.BallColor.Purple);
            colorPattern.add(Sorter.BallColor.Green);
            colorPattern.add(Sorter.BallColor.Purple);
            colorPattern.add(Sorter.BallColor.Purple);
        }

        else if (id == 22) {
            //PGP
            colorOrder = Eye.ColorOrder.PGP;

            //Launch Purple1, Counter Clockwise to Green, 2 Clockwise to Purple2
            launchPattern.add(0);
            launchPattern.add(-1);
            launchPattern.add(2);
            //
            launchPattern.add(0);
            launchPattern.add(-1);
            launchPattern.add(2);
            //
            launchPattern.add(0);
            launchPattern.add(-1);
            launchPattern.add(2);
            //
            colorPattern.add(Sorter.BallColor.Purple);
            colorPattern.add(Sorter.BallColor.Green);
            colorPattern.add(Sorter.BallColor.Purple);
            colorPattern.add(Sorter.BallColor.Purple);
            colorPattern.add(Sorter.BallColor.Green);
            colorPattern.add(Sorter.BallColor.Purple);
            colorPattern.add(Sorter.BallColor.Purple);
            colorPattern.add(Sorter.BallColor.Green);
            colorPattern.add(Sorter.BallColor.Purple);
        }
        // Always provide at least an order
        else{
            //PPG
            colorOrder = Eye.ColorOrder.PPG;

            //Clockwise to Purple2, Counter Clockwise to Purple1, Counter Clockwise to Green
            launchPattern.add(1);
            launchPattern.add(-1);
            launchPattern.add(-1);
            //
            launchPattern.add(1);
            launchPattern.add(-1);
            launchPattern.add(-1);
            //
            launchPattern.add(1);
            launchPattern.add(-1);
            launchPattern.add(-1);
            //
            colorPattern.add(Sorter.BallColor.Purple);
            colorPattern.add(Sorter.BallColor.Purple);
            colorPattern.add(Sorter.BallColor.Green);
            colorPattern.add(Sorter.BallColor.Purple);
            colorPattern.add(Sorter.BallColor.Purple);
            colorPattern.add(Sorter.BallColor.Green);
            colorPattern.add(Sorter.BallColor.Purple);
            colorPattern.add(Sorter.BallColor.Purple);
            colorPattern.add(Sorter.BallColor.Green);
        }

        // Save color order for teleop
        SharedPreferences prefs = hardwareMap.appContext.getSharedPreferences("DecodeData", Context.MODE_PRIVATE);
        SharedPreferences.Editor editor = prefs.edit();
        editor.putInt("colorOrder", colorOrder.getId());
        editor.apply();

        // Voltage sets min power for pickup speed
        double currentVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {           // Ignore sensors that return 0
                currentVoltage = Math.min(currentVoltage, voltage);
            }
        }

        if (currentVoltage < MIN_PICKUP_VOLTAGE) {
            pickupPower = MAX_PICKUP_POWER;
        } else if (currentVoltage > MAX_PICKUP_VOLTAGE) {
            pickupPower = MIN_PICKUP_POWER;
        } else {
            double deltaVoltage = MAX_PICKUP_VOLTAGE-MIN_PICKUP_VOLTAGE;
            double myFraction = (currentVoltage-MIN_PICKUP_VOLTAGE)/deltaVoltage;
            double deltaPower = (MAX_PICKUP_POWER-MIN_PICKUP_POWER);
            pickupPower = MAX_PICKUP_POWER-(myFraction * deltaPower);
        }


        // Start all body parts (except the eye, which has already started)
        for( Thread part : partList)
            if(!part.equals(eye))
                part.start();
    }

    @Override public void init_loop() {
        // Watch obelisk for april tag
        int id = eye.getFiducialID();
        Eye.ColorOrder co = Eye.ColorOrder.fromId(id);
        telemetry.addData("ID", id);
        telemetry.addData("Color Order", co.toString());
        telemetry.update();
    }

    @Override public void loop() {
        // These loop the movements of the robot
        eye.setMode(Eye.Mode.NONE);
        motion.follower.update();
        autonomousPathUpdate();

        // Update telemetry
        super.partList.forEach(part -> part.getTelemetry(telemetry));
        if(super.partList.size()>0) telemetry.update();
    }
}