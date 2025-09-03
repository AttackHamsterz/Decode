package org.firstinspires.ftc.teamcode;

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

    protected final Follower follower;
    protected static final double CLOSE_X_IN = 1.0;
    protected static final double CLOSE_Y_IN = 1.0;
    protected static final double CLOSE_ANG_RAD = Math.toRadians(5.0);

    /**
     * Setup the motion object
     * @param ssom access to the standard setup op mode
     */
    public Motion(StandardSetupOpMode ssom){
        this.ssom = ssom;
        this.gamepad = ssom.gamepad1;
        follower = Constants.createFollower(ssom.hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    @Override
    public void run() {
        // Test to watch amperage protection
        //DcMotorEx leftFront = ssom.hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        //DcMotorEx leftRear = ssom.hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        //DcMotorEx rightRear = ssom.hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        //DcMotorEx rightFront = ssom.hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);

        // Precise control
        //leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //double leftCurrent_ma = leftFront.getCurrent(CurrentUnit.MILLIAMPS);

        if (!ignoreGamepad) {
            // Only start teleop drive if the gamepad is enabled
            follower.startTeleopDrive();

            while (!isInterrupted()) {
                // Update Pedro to move the robot based on:
                follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, true);
                follower.update();
            }

            // Short sleep to keep this loop from saturating
            //try {
            //    sleep(LOOP_PAUSE_MS);
            //} catch (InterruptedException e) {
            //    interrupt();
            //}
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
