package org.firstinspires.ftc.teamcode;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
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

    // Poses used in the super auto: x, y, and heading (in Radians).
    // Pedro uses 0 - 144 for x and y, with 0, 0 being the bottom left of the field.
    // This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
    private final Pose startPose = new Pose(0, 72, Math.toRadians(0));
    private final Pose scorePose = new Pose(23, 72, Math.toRadians(0));
    private final Pose parkControlPose = new Pose(10, 97, Math.toRadians(0));
    private final Pose parkPose = new Pose(5, 122, Math.toRadians(0));

    // Path and path-chain objects
    private Path scorePath, parkPath;

    // Build paths for the auto before auto starts
    public void buildPaths() {

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

        // Score first element
        scorePath = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePath.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        //scorePath.setConstantInterpolation(startPose.getHeading());

        // Park with a bezier curve
        parkPath = new Path(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)));
        parkPath.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Begin the autonomous
                motion.follower.followPath(scorePath);
                setPathState(1);
                break;
            case 1:

                // You could check for
                //  Follower State: "if(!follower.isBusy() {}"
                //  Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                //  Robot Position: "if(follower.getPose().getX() > 36) {}"

                // Wait until the robot position is close (1 inch away) from the scorePose position
                if(!motion.follower.isBusy()) {
                    // Code to score here

                    // Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    motion.follower.followPath(parkPath,true);
                    setPathState(2);
                }
                break;
            case 2:
                // Case waits for parking
                if(!motion.follower.isBusy()) {
                    // Parked, we're done
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // @Override
    public void runOpMode() throws InterruptedException {

        // Init
        pathTimer = new Timer();
        buildPaths();
        setPathState(0);

        // Complete setup and wait for play
        super.runOpMode();

        // Run until state machine completes
        while (pathState>=0) {
            // These loop the movements of the robot
            motion.follower.update();
            autonomousPathUpdate();
        }
    }
}