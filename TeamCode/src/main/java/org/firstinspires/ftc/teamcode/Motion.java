package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Class to provide robot motion.  Under the hood it's just using the pedro pathing follower
 */
public class Motion extends RobotPart<MotionMetric>{

    public Follower follower;
    protected static final double CLOSE_X_IN = 1.0;
    protected static final double CLOSE_Y_IN = 1.0;
    protected static final double CLOSE_ANG_RAD = Math.toRadians(5.0);

    // Allow the eye to control the robot yaw
    private double externalTurn;

    /**
     * Setup the motion object
     * @param ssom access to the standard setup op mode
     */
    public Motion(StandardSetupOpMode ssom){
        this.ssom = ssom;
        PanelsConfigurables.INSTANCE.refreshClass(this);
        follower = Constants.createFollower(ssom.hardwareMap);
        externalTurn = 0;
    }

    /**
     * Allows a third party to turn the robot, great for smoothly aiming at the fiducial
     * @param turn [-1, 1] to apply to joystick
     */
    public void setTurn(double turn){
        externalTurn = turn;
    }

    @Override
    public void run() {

        if (!ssom.gamepadBuffer.ignoreGamepad) {
            // Only start teleop drive if the gamepad is enabled
            follower.startTeleopDrive( );
            follower.update();
            setRunning();
            while (running) {
                // Scale the speed
                float scale = 1.0f;
                if(ssom.gamepadBuffer.g1LeftBumper)
                    scale = 0.5f;
                else if(ssom.gamepadBuffer.g1LeftTrigger > 0.05)
                    scale = 1.0f - ssom.gamepadBuffer.g1LeftTrigger * 0.75f;

                // Update Pedro to move the robot based on stick input:
                follower.setTeleOpDrive(-ssom.gamepadBuffer.g1LeftStickY*scale, -ssom.gamepadBuffer.g1LeftStickX*scale, (-externalTurn-ssom.gamepadBuffer.g1RightStickX)*scale, true);
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
            follower.update();
        }
    }

    @Override
    public void setTo(MotionMetric metric){
        //Pose startPose = follower.getPose();
        //Path path = new Path(new BezierLine(startPose, metric.pose));
        //path.setLinearHeadingInterpolation(startPose.getHeading(), metric.pose.getHeading());
        //PathChain pathChain = new PathChain(path);
        //follower.followPath(pathChain, metric.power, true);
    }

    @Override
    public void safeHold() {
        //follower.holdPoint(follower.getPose());
    }

    @Override
    protected boolean closeEnough(MotionMetric metric){
        //return follower.atPose(metric.pose, CLOSE_X_IN, CLOSE_Y_IN, CLOSE_ANG_RAD);
        return false;
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        if(DEBUG) {
            Pose pose = follower.getPose();
            if(pose != null) {
                telemetry.addData("X", pose.getX());
                telemetry.addData("Y", pose.getY());
                telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
            }
        }
    }
}
