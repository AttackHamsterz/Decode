package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * This super auto entry point runs everything that all auto's will need
 */
@Autonomous(name = "Auto", group = "Robot")
@Disabled
public class AutoOpMode extends StandardSetupOpMode {


    protected Timer pathTimer;

    // Finite state machine state
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
        //pathTimer.resetTimer();
        //telemetry.addData("Current Path State:", pathState);
        //telemetry.update();
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

        // Start all body parts (except the eye, which has already started)
        for( Thread part : partList)
            if(!part.equals(eye))
                part.start();
    }

    @Override public void init_loop() {
        // Watch obelisk for april tag
        telemetry.addData("ID", eye.getFiducialID());
        telemetry.addData("Color Order", eye.getColorOrder());
        telemetry.update();


    }

    @Override public void loop() {
        // These loop the movements of the robot
        motion.follower.update();
        autonomousPathUpdate();

        telemetry.addData("PathState", pathState);
        telemetry.addData("Pose", motion.follower.getPose().toString());
    }
}