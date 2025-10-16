package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Class to provide robot motion.  Under the hood it's just using the pedro pathing follower
 */
public class Motion extends RobotPart<MotionMetric>{

    public static Follower follower;
    protected static final double CLOSE_X_IN = 1.0;
    protected static final double CLOSE_Y_IN = 1.0;
    protected static final double CLOSE_ANG_RAD = Math.toRadians(5.0);

    /**
     * Setup the motion object
     * @param ssom access to the standard setup op mode
     */
    public Motion(StandardSetupOpMode ssom, boolean ignoreGamepad){
        setIgnoreGamepad(ignoreGamepad);
        this.ssom = ssom;
        if(follower == null)
            PanelsConfigurables.INSTANCE.refreshClass(this);
        follower = Constants.createFollower(ssom.hardwareMap);
        follower.setStartingPose(new Pose());
    }

    @Override
    public void run() {

        if (!ignoreGamepad) {
            // Only start teleop drive if the gamepad is enabled
            follower.startTeleopDrive( );
            follower.update();

            while (!isInterrupted()) {
                // Update Pedro to move the robot based on:
                follower.setTeleOpDrive(-ssom.gamepadBuffer.g1LeftStickY, -ssom.gamepadBuffer.g1LeftStickX, -ssom.gamepadBuffer.g1RightStickX, true);
                follower.update();

                // Short sleep to keep this loop from saturating
                //try {
                //    sleep(LOOP_PAUSE_MS);
                //} catch (InterruptedException e) {
                //    interrupt();
                //}
            }

            // Stop all motion
            follower.startTeleopDrive(true);
            follower.setTeleOpDrive(0,0,0,true);
        }
    }

    @Override
    public void setTo(MotionMetric metric){
        Pose startPose = follower.getPose();
        Path path = new Path(new BezierLine(startPose, metric.pose));
        path.setLinearHeadingInterpolation(startPose.getHeading(), metric.pose.getHeading());
        PathChain pathChain = new PathChain(path);
        follower.followPath(pathChain, metric.power, true);
    }

    @Override
    public void safeHold() {
        follower.holdPoint(follower.getPose());
    }

    @Override
    protected boolean closeEnough(MotionMetric metric){
        return follower.atPose(metric.pose, CLOSE_X_IN, CLOSE_Y_IN, CLOSE_ANG_RAD);
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        Pose pose = follower.getPose();
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
    }
}
