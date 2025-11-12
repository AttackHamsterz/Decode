package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto: Board", group = "Robot")
@Disabled
public class BoardAutoOpMode extends AutoOpMode{

    private static final double FIRST_LAUNCH_RPM = 2500.00;
    private Pose startPose;
    private Pose initialScorePose;
    private Pose midLinePose;
    private Pose firstLineStart;
    private Pose firstLineEnd;
    private Pose parkPose;
    private Pose secondLineStart;
    private Pose secondLineEnd;

    private PathChain boardToScorePath;
    private PathChain scoreToFirstLinePath;
    private PathChain firstLineEndPath;
    private PathChain firstLineEndToScore;
    private PathChain scoreToPark;
    private PathChain scoreToSecondLine;
    private PathChain secondLineEndPath;
    private PathChain secondLineEndToScore;

    @Override public void init() {
        final double centerLineX = 72.0;
        final double startPoseX = 56.0;
        final double initialScorePoseX = 44.22;
        final double midLinePoseX = 30.0;
        final double lineStartX = 34.0;
        final double lineEndX = 48.92;
        final double parkX = 49.0;

        startPose = new Pose((color == COLOR.BLUE) ? 0 :centerLineX+startPoseX, 0, Math.toRadians((color == COLOR.BLUE) ? 0 : 180));
        initialScorePose = new Pose((color == COLOR.BLUE) ? 9.78 :centerLineX+initialScorePoseX, -0.5, Math.toRadians((color == COLOR.BLUE) ? 135 : 45));
        midLinePose = new Pose ((color == COLOR.BLUE) ? 24 :centerLineX+midLinePoseX, -13.5, Math.toRadians((color == COLOR.BLUE) ? 135 : 45));
        firstLineStart = new Pose((color == COLOR.BLUE) ? 20 :centerLineX+lineStartX,-31.5, Math.toRadians((color == COLOR.BLUE) ? 180 : 0));
        firstLineEnd = new Pose((color == COLOR.BLUE) ? 5.08 :centerLineX+lineEndX,-31.5, Math.toRadians((color == COLOR.BLUE) ? 180 : 0));
        parkPose = new Pose( (color == COLOR.BLUE) ? 5 :centerLineX+parkX, -46, Math.toRadians((color == COLOR.BLUE) ? 0 : 180));
        secondLineStart = new Pose((color == COLOR.BLUE) ? 20 :centerLineX+lineStartX, -55, Math.toRadians((color == COLOR.BLUE) ? 180 : 0));
        secondLineEnd = new Pose((color == COLOR.BLUE) ? 5.08 :centerLineX+lineEndX, -55, Math.toRadians((color == COLOR.BLUE) ? 180 : 0));

        super.init();

        motion.follower.setStartingPose(startPose);
        telemetry.addData("Auto Pose", startPose);
        telemetry.update();
        ballLifter.lift();
    }

    @Override
    public void buildPaths() {
        boardToScorePath = motion.follower.pathBuilder()
                .addPath(new BezierLine(startPose, initialScorePose))
                .setLinearHeadingInterpolation(firstLineStart.getHeading(), firstLineEnd.getHeading())
                .build();
        scoreToFirstLinePath = motion.follower.pathBuilder()
                .addPath(new BezierCurve(initialScorePose, midLinePose, firstLineStart))
                .setLinearHeadingInterpolation(initialScorePose.getHeading(), midLinePose.getHeading(), firstLineStart.getHeading())
                .build();
        firstLineEndPath = motion.follower.pathBuilder()
                .addPath(new BezierLine(firstLineStart, firstLineEnd))
                .setLinearHeadingInterpolation(firstLineStart.getHeading(), firstLineEnd.getHeading())
                .build();
        firstLineEndToScore = motion.follower.pathBuilder()
                .addPath(new BezierLine(firstLineEnd, initialScorePose))
                .setLinearHeadingInterpolation(firstLineEnd.getHeading(), initialScorePose.getHeading())
                .build();
        scoreToSecondLine = motion.follower.pathBuilder()
                .addPath(new BezierLine(initialScorePose, secondLineStart))
                .setLinearHeadingInterpolation(initialScorePose.getHeading(), secondLineStart.getHeading())
                .build();
        secondLineEndPath = motion.follower.pathBuilder()
                .addPath(new BezierLine(motion.follower::getPose, secondLineEnd))
                .setLinearHeadingInterpolation(secondLineStart.getHeading(), secondLineEnd.getHeading())
                .build();
        secondLineEndToScore = motion.follower.pathBuilder()
                .addPath(new BezierLine(secondLineEnd, initialScorePose))
                .setLinearHeadingInterpolation(secondLineEnd.getHeading(), initialScorePose.getHeading())
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
                // Follow first path for initial shots
                motion.follower.followPath(boardToScorePath,0.5, true);

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
            // Third 3 balls
            case 17:
            case 19:
            case 21:
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
            case 18:
            case 20:
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
                    motion.follower.followPath(scoreToFirstLinePath, PATH_VELOCITY_PERCENTAGE, true);
                    incrementPathState();
                }
                break;
            case 7:
                // Pick up balls
                if(!motion.follower.isBusy()){
                    motion.follower.followPath(firstLineEndPath, PICKUP_VELOCITY_PERCENTAGE, false);
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
                    motion.follower.followPath(firstLineEndToScore, PATH_VELOCITY_PERCENTAGE, true);
                    incrementPathState();
                }
                break;
            case 14:
                // Are we done lifting?
                if(!ballLifter.isLifting()){
                    // Slow down the launcher
                    launcher.setVelocityRPM(0);

                    // Start front intake
                    intake.frontIntakeOn();

                    // Drive to pick up first line of balls
                    motion.follower.followPath(scoreToSecondLine, PATH_VELOCITY_PERCENTAGE, false);
                    incrementPathState();
                }
                break;
            case 15:
                //Pick up balls
                if(!motion.follower.isBusy()) {
                    motion.follower.followPath(secondLineEndPath, PICKUP_VELOCITY_PERCENTAGE, false);
                    incrementPathState();
                }
                break;
            case 16:
                //Drive to score next round of balls
                if(!motion.follower.isBusy()){
                    // Stop intake
                    intake.frontIntakeStop();

                    // TODO - Rotate to first color of pattern

                    // Spin up launcher
                    launcher.setVelocityRPM(FIRST_LAUNCH_RPM);

                    // Drive to score
                    motion.follower.followPath(secondLineEndToScore, PATH_VELOCITY_PERCENTAGE, true);
                    incrementPathState();
                }
                break;
            case 22:
                // Park
                if(!ballLifter.isLifting()){
                    // Stop the launcher
                    launcher.setVelocityRPM(0);

                    // Drive to park
                    motion.follower.followPath(scoreToPark, PATH_VELOCITY_PERCENTAGE, true);
                    incrementPathState();
                }
                break;
            case 23:
                if(!motion.follower.isBusy()) {
                    setPathState(-1);
                }
        }
    }
}