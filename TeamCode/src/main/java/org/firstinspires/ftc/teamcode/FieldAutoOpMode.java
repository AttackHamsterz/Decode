package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto: Field", group = "Robot")
@Disabled
public class FieldAutoOpMode extends AutoOpMode {
    private static final double FIRST_LAUNCH_RPM = 3200.00;
    private PathChain startToScorePath;

    private Pose startPose;
    private Pose initialScorePose;

    @Override
    public void init() {

        final double centerLineX = 72.0;
        final double startPoseX = 8.0;
        startPose = new Pose((color == COLOR.BLUE) ? centerLineX - startPoseX : centerLineX + startPoseX, 8.5, Math.toRadians(90));
        initialScorePose = new Pose((color == COLOR.BLUE) ? centerLineX - startPoseX : centerLineX + startPoseX, 80, Math.toRadians(135));

        super.init();

        motion.follower.setStartingPose(startPose);
        telemetry.addData("Auto Pose", startPose);
        telemetry.update();
        ballLifter.lift();
    }

    @Override

    public void buildPaths() {
        startToScorePath = motion.follower.pathBuilder()
                .addPath(new BezierLine(startPose, initialScorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), initialScorePose.getHeading())
                .build();
                setPathState(0);

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                motion.follower.followPath(startToScorePath, 0.5,true);
                // TODO - Rotate to first color of pattern
                // Loaded green left, purple back/right
                sorter.rotateLeftToLaunch();

                // Spin up launcher
                launcher.setVelocityRPM(FIRST_LAUNCH_RPM);
                incrementPathState();
                break;
            // First 3 balls
            case 1:
            case 3:
            case 5:
                // Done driving, sorter ready, launcher ready, lifter reset?
                if(!motion.follower.isBusy() && !sorter.isSpinning() && launcher.launchReady() && ballLifter.isReset()) {
                    // Launch
                    try {
                        Thread.sleep(300);
                    } catch (InterruptedException ignore) {
                    }
                    ballLifter.lift();
                    incrementPathState();
                }
                break;
                // Second 3 balls
            case 2:
            case 4:
            case 6:
                // Are we done lifting?
                if(!ballLifter.isLifting()){
                    // Slow down the launcher
                    launcher.setVelocityRPM(0);

                    // Start front intake
                    intake.frontIntakeOn();

                    // Drive to pick up first line of balls
                    //motion.follower.followPath(scoreToFirstLinePath, PATH_VELOCITY_PERCENTAGE, true);
                    incrementPathState();
                }
                break;
            case 9:
            case 11:
            case 13:
            case 14:
                if(!motion.follower.isBusy()) {
                    setPathState(-1);
                }
        }
    }
}
