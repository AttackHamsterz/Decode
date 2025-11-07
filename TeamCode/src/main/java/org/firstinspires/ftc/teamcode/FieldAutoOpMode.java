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
    private Pose startPose;
    private Pose initialScorePose;
    private Pose thirdLineStart;
    private Pose thirdLineEnd;
    private Pose parkPose;


    private PathChain startToScorePath;
    private PathChain scoreToThirdLinePath;
    private PathChain thirdLineEndPath;
    private PathChain thirdLineEndToScore;
    private PathChain scoreToPark;


    @Override
    public void init() {

        final double centerLineX = 72.0;
        final double startPoseX = 8.0;
        final double scorePoseX = 15;
        final double lineStartX = 30.0;
        final double lineEndX = 48.92;
        final double parkPoseX = 36.0;

        startPose = new Pose((color == COLOR.BLUE) ? centerLineX - startPoseX : centerLineX + startPoseX, 8.5, Math.toRadians(90));
        initialScorePose = new Pose((color == COLOR.BLUE) ? centerLineX - scorePoseX : centerLineX + scorePoseX, 84, Math.toRadians(135));
        thirdLineStart = new Pose((color == COLOR.BLUE) ? centerLineX - lineStartX : centerLineX + lineStartX, 35, Math.toRadians((color == COLOR.BLUE) ? 180 : 0));
        thirdLineEnd = new Pose((color == COLOR.BLUE) ? centerLineX - lineEndX :centerLineX + lineEndX, 35, Math.toRadians((color == COLOR.BLUE) ? 180 : 0));
        parkPose = new Pose ((color == COLOR.BLUE) ? centerLineX - parkPoseX : centerLineX + parkPoseX, 10, Math.toRadians((color == COLOR.BLUE) ? 180 : 0));

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
        scoreToThirdLinePath = motion.follower.pathBuilder()
                .addPath(new BezierLine(initialScorePose, thirdLineStart))
                .setLinearHeadingInterpolation(initialScorePose.getHeading(), thirdLineStart.getHeading())
                .build();
        thirdLineEndPath = motion.follower.pathBuilder()
                .addPath(new BezierLine(motion.follower::getPose, thirdLineEnd))
                .setLinearHeadingInterpolation(thirdLineStart.getHeading(), thirdLineEnd.getHeading())
                .build();
        thirdLineEndToScore = motion.follower.pathBuilder()
                .addPath(new BezierLine(thirdLineEnd, initialScorePose))
                .setLinearHeadingInterpolation(thirdLineEnd.getHeading(), initialScorePose.getHeading())
                .build();
        scoreToPark = motion.follower.pathBuilder()
                .addPath(new BezierLine(initialScorePose, parkPose))
                .setLinearHeadingInterpolation(initialScorePose.getHeading(), parkPose.getHeading())
                .build();
        setPathState(0);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                motion.follower.followPath(startToScorePath, 0.7,true);
                // TODO - Rotate to first color of pattern
                // Loaded green left, purple back/right
                sorter.rotateClockwise(launchPattern.get(launchIndex++));

                // Spin up launcher
                launcher.setVelocityRPM(FIRST_LAUNCH_RPM);
                incrementPathState();
                break;
            // First 3 balls
            case 1:
            case 3:
            case 5:
            // Second 3 balls
            case 9:
            case 11:
            case 13:
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

            case 2:
            case 4:
            case 10:
            case 12:
                // Are we done lifting?
                if(!ballLifter.isLifting()){
                    // TODO - Rotate to next color of pattern
                    sorter.rotateClockwise(launchPattern.get(launchIndex++));
                    incrementPathState();
                }
                break;
            case 6:
                // Are we done lifting?
                if(!ballLifter.isLifting()){
                    // Slow down the launcher
                    launcher.setVelocityRPM(0);

                    // Start front intake
                    intake.frontIntakeOn();

                    // Drive to pick up first line of balls
                    motion.follower.followPath(scoreToThirdLinePath, PATH_VELOCITY_PERCENTAGE, true);
                    incrementPathState();
                }
                break;
            case 7:
                // Pick up balls
                if(!motion.follower.isBusy()){
                    motion.follower.followPath(thirdLineEndPath, PICKUP_VELOCITY_PERCENTAGE, false);
                    incrementPathState();
                }
                break;
            case 8:
                // Drive to launch again
                if(!motion.follower.isBusy()){
                    // Stop front intake
                    intake.frontIntakeStop();

                    // TODO - Rotate to first color of pattern

                    // Spin up launcher
                    launcher.setVelocityRPM(FIRST_LAUNCH_RPM);

                    // Drive to score
                    motion.follower.followPath(thirdLineEndToScore, PATH_VELOCITY_PERCENTAGE, true);
                    incrementPathState();
                }
                break;

            case 14:
                // Park
                if(!ballLifter.isLifting()){
                    // Stop the launcher
                    launcher.setVelocityRPM(0);

                    // Drive to park
                    motion.follower.followPath(scoreToPark, PATH_VELOCITY_PERCENTAGE, true);
                    incrementPathState();
                }
                break;
            case 15:
                if(!motion.follower.isBusy()) {
                    setPathState(-1);
                }
        }
    }
}
