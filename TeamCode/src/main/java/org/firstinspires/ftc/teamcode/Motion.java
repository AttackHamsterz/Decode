package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * Class to provide robot motion.  Under the hood it's just using the pedro pathing follower
 */
public class Motion extends RobotPart<MotionMetric>{

    protected final Follower follower;

    /**
     * Setup the motion object
     * @param ssom access to the standard setup op mode
     */
    public Motion(StandardSetupOpMode ssom){
        this.ssom = ssom;
        this.gamepad = ssom.gamepad1;
        follower = new Follower(ssom.hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    @Override
    public void run() {
        follower.startTeleopDrive();

        // Test to watch amperage protection
        DcMotorEx leftFront = ssom.hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        DcMotorEx leftRear = ssom.hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        DcMotorEx rightRear = ssom.hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        DcMotorEx rightFront = ssom.hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //double leftCurrent_ma = leftFront.getCurrent(CurrentUnit.MILLIAMPS);

        while (!isInterrupted()) {
            if (!ignoreGamepad) {
                // Update Pedro to move the robot based on:
                follower.setTeleOpMovementVectors(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, true);
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
    public void setTo(MotionMetric metric){
        Pose startPose = follower.getPose();
        Path path = new Path(new BezierLine(new Point(startPose), new Point(metric.pose)));
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
        Pose currentPose = follower.getPose();
        double deltaHeading = Math.abs(currentPose.getHeading() - metric.pose.getHeading());
        double deltaX = Math.abs(currentPose.getX() - metric.pose.getX());
        double deltaY = Math.abs(currentPose.getY() - metric.pose.getY());
        return deltaHeading < FollowerConstants.pathEndHeadingConstraint &&
                deltaX < FollowerConstants.pathEndTranslationalConstraint &&
                deltaY < FollowerConstants.pathEndTranslationalConstraint;
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        Pose pose = follower.getPose();
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
    }
}
