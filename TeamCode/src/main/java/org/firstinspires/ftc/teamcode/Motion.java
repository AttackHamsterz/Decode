package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * Class to provide robot motion.  Under the hood it's just using the pedro pathing follower
 */
public class Motion extends RobotPart{

    protected final Follower follower;

    /**
     * Setup the motion object
     * @param ssom access to the standard setup op mode
     */
    public Motion(StandardSetupOpMode ssom){
        this.ssom = ssom;
        follower = new Follower(ssom.hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    @Override
    public void run() {
        follower.startTeleopDrive();

        while (!isInterrupted()) {
            if (!ignoreGamepad) {
                // Update Pedro to move the robot based on:
                //  Forward/Backward Movement: -gamepad1.left_stick_y
                //  Left/Right Movement: -gamepad1.left_stick_x
                //  Turn Left/Right Movement: -gamepad1.right_stick_x
                //  Robot-Centric Mode: false
                follower.setTeleOpMovementVectors(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, false);
                follower.update();
            }

            // Short sleep to keep this loop from saturating
            try {
                sleep(LOOP_PAUSE_MS);
            } catch (InterruptedException e) {
                interrupt();
            }
        }
    }

    @Override
    public void safeHold(int position) {

    }

    @Override
    public int getCurrentPosition() {
        return 0;
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        Pose pose = follower.getPose();
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
    }
}
