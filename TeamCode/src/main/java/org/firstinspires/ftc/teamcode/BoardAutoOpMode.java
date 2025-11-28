package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto: Board", group = "Robot")
@Disabled
public class BoardAutoOpMode extends AutoOpMode{
    protected final double PICKUP_VELOCITY_PERCENTAGE = 0.20;
    private static final double FIRST_LAUNCH_RPM = 2500.00; // Launcher speed
    private static final int SHOT_DELAY_MS = 50;            // Ball settle time
    private static final int LINE_END_DELAY_MS = 1050;      // Wait after line to rotate to color

    private Pose startPose;
    private Pose initialScorePose;
    private Pose secondScorePose;
    private Pose midLinePose;
    private Pose firstLineStart;
    private Pose firstLineEnd;
    private Pose parkPose;
    private Pose secondLineStart;
    private Pose secondLineEnd;

    private Path boardToScorePath;
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
        final double startPoseY = 113.5;
        final double initialScorePoseX = 44.22;
        final double initialScorePoseY = 113.0;
        final double secondScorePoseX = 37;
        final double secondScorePoseY = 121;
        final double midLinePoseX = 25.0;
        final double midLinePoseY = 100.0;
        final double firstlineStartX = 28.5;
        final double secondLineStartX = 29.5;
        final double firstLineStartY = 85.5;
        final double secondLineStartY = 61.0;
        final double lineEndX = 53.0;
        final double parkX = 49.0;
        final double parkY = 67.5;

        startPose = new Pose((color == COLOR.BLUE) ? centerLineX-startPoseX :centerLineX+startPoseX, startPoseY, Math.toRadians((color == COLOR.BLUE) ? 0 : 180));
        initialScorePose = new Pose((color == COLOR.BLUE) ? centerLineX-initialScorePoseX :centerLineX+initialScorePoseX, initialScorePoseY, Math.toRadians((color == COLOR.BLUE) ? 135 : 45));
        secondScorePose = new Pose((color == COLOR.BLUE) ? centerLineX-secondScorePoseX :centerLineX+secondScorePoseX, secondScorePoseY, Math.toRadians((color == COLOR.BLUE) ? 155 : 25));
        midLinePose = new Pose ((color == COLOR.BLUE) ? centerLineX-midLinePoseX :centerLineX+midLinePoseX, midLinePoseY, Math.toRadians((color == COLOR.BLUE) ? 180 : 0));
        firstLineStart = new Pose((color == COLOR.BLUE) ? centerLineX-firstlineStartX :centerLineX+firstlineStartX,firstLineStartY, Math.toRadians((color == COLOR.BLUE) ? 180 : 0));
        firstLineEnd = new Pose((color == COLOR.BLUE) ? centerLineX-lineEndX :centerLineX+lineEndX,firstLineStartY, Math.toRadians((color == COLOR.BLUE) ? 180 : 0));
        parkPose = new Pose( (color == COLOR.BLUE) ? centerLineX-parkX :centerLineX+parkX, parkY, Math.toRadians((color == COLOR.BLUE) ? 0 : 180));
        secondLineStart = new Pose((color == COLOR.BLUE) ? centerLineX-secondLineStartX :centerLineX+secondLineStartX, secondLineStartY, Math.toRadians((color == COLOR.BLUE) ? 180 : 0));
        secondLineEnd = new Pose((color == COLOR.BLUE) ? centerLineX-lineEndX :centerLineX+lineEndX, secondLineStartY, Math.toRadians((color == COLOR.BLUE) ? 180 : 0));

        // Setup and build paths
        super.init();
        motion.follower.setStartingPose(startPose);
        ballLifter.lift();
    }

    @Override
    public void buildPaths() {
        boardToScorePath = new Path(new BezierLine(startPose, initialScorePose));
        boardToScorePath.setLinearHeadingInterpolation(startPose.getHeading(), initialScorePose.getHeading());
        motion.follower.pathBuilder()
                .addPath(new BezierLine(startPose, initialScorePose))
                .setLinearHeadingInterpolation(firstLineStart.getHeading(), firstLineEnd.getHeading())
                .build();
        scoreToFirstLinePath = motion.follower.pathBuilder()
                .addPath(new BezierCurve(initialScorePose, midLinePose, firstLineStart))
                .setLinearHeadingInterpolation(initialScorePose.getHeading(), midLinePose.getHeading(), firstLineStart.getHeading())
                .build();
        firstLineEndPath = motion.follower.pathBuilder()
                .addPath(new BezierLine(motion.follower::getPose, firstLineEnd))
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
                .addPath(new BezierLine(secondLineEnd, secondScorePose))
                .setLinearHeadingInterpolation(secondLineEnd.getHeading(), secondScorePose.getHeading())
                .build();
        scoreToPark = motion.follower.pathBuilder()
                .addPath(new BezierLine(secondScorePose, parkPose))
                .setLinearHeadingInterpolation(secondScorePose.getHeading(), parkPose.getHeading())
                .build();
        setPathState(0);

    }

    private void delayedColorQueue(Sorter.BallColor color, int msDelay){
        // Schedules a sorter rotation for the future
        // We try to launch the tasked color first, but will queue the other color if not found
        ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);
        Runnable task = () -> {
            if(color == Sorter.BallColor.Green) {
                if(!sorter.rotateGreenToLaunch())
                    sorter.rotatePurpleToLaunch();
            }
            else {
                if(!sorter.rotatePurpleToLaunch())
                    sorter.rotateGreenToLaunch();
            }
        };
        scheduler.schedule(task, msDelay, TimeUnit.MILLISECONDS);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Follow first path for initial shots
                motion.follower.followPath(boardToScorePath, true);

                // Loaded purple left and back and green right when looking at robot
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
            case 10:
            case 12:
            case 14:
            // Third 3 balls
            case 19:
            case 21:
            case 23:
                // Done driving, sorter ready, launcher ready, lifter reset?
                if(!motion.follower.isBusy() && sorter.isNotSpinning() && launcher.launchReady() && ballLifter.isReset()) {

                    // Launch (delay let's ball settle from rotation)
                    try {
                        Thread.sleep(SHOT_DELAY_MS);
                    } catch (InterruptedException ignore) {
                    }
                    ballLifter.lift();
                    incrementPathState();
                }
                break;
            case 2:
            case 4:
                // Are we done lifting?
                if(!ballLifter.isLifting()){
                    sorter.rotateClockwise(launchPattern.get(launchIndex++));
                    incrementPathState();
                }
                break;
            case 11:
            case 13:
            case 20:
            case 22:
                // Are we done lifting?
                if(!ballLifter.isLifting()){
                    if(colorPattern.get(launchIndex++) == Sorter.BallColor.Green) {
                        if(!sorter.rotateGreenToLaunch())
                            sorter.rotatePurpleToLaunch();
                    }
                    else {
                        if(!sorter.rotatePurpleToLaunch())
                            sorter.rotateGreenToLaunch();
                    }
                    incrementPathState();
                }
                break;
            case 6:
                // Are we done lifting?
                if(!ballLifter.isLifting()){
                    // Slow down the launcher
                    launcher.setVelocityRPM(0);

                    // Drive to pick up first line of balls
                    motion.follower.followPath(scoreToFirstLinePath, PATH_VELOCITY_PERCENTAGE, false);
                    incrementPathState();
                }
                break;
            case 7:
                // Pick up balls
                if(!motion.follower.isBusy()){
                    // Start front intake
                    intake.frontIntakeOn();
                    sorter.autoTurnOn();

                    // Drive to path end
                    motion.follower.followPath(firstLineEndPath, pickupPower, false);
                    incrementPathState();
                }
                break;
            case 8:
                // Drive to launch again
                if(!motion.follower.isBusy()){

                    // Spin up launcher
                    launcher.setVelocityRPM(FIRST_LAUNCH_RPM);

                    // Drive to score
                    motion.follower.followPath(firstLineEndToScore, PATH_VELOCITY_PERCENTAGE, true);
                    incrementPathState();
                }
                break;
            case 9:
                if(!motion.follower.isBusy()){
                    try {
                        Thread.sleep(LINE_END_DELAY_MS);
                    } catch (InterruptedException ignore) {}

                    intake.frontIntakeStop();
                    sorter.autoTurnOff();

                    // After end of line delay spinning to color for 1 second (finish intake)
                    delayedColorQueue(colorPattern.get(launchIndex++), 0);
                    incrementPathState();
                }
                break;
            case 18:
                if(!motion.follower.isBusy()){
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException ignore) {}

                    intake.frontIntakeStop();
                    sorter.autoTurnOff();

                    // After end of line delay spinning to color for 1 second (finish intake)
                    delayedColorQueue(colorPattern.get(launchIndex++), 0);
                    incrementPathState();
                }
                break;
            case 15:
                // Are we done lifting?
                if(!ballLifter.isLifting()){
                    // Slow down the launcher
                    launcher.setVelocityRPM(0);

                    // Drive to pick up first line of balls
                    motion.follower.followPath(scoreToSecondLine, PATH_VELOCITY_PERCENTAGE, false);
                    incrementPathState();
                }
                break;
            case 16:
                //Pick up balls
                if(!motion.follower.isBusy()) {
                    // Start front intake
                    intake.frontIntakeOn();
                    sorter.autoTurnOn();

                    motion.follower.followPath(secondLineEndPath, pickupPower, false);
                    incrementPathState();
                }
                break;
            case 17:
                //Drive to score next round of balls
                if(!motion.follower.isBusy()){

                    // Spin up launcher
                    launcher.setVelocityRPM(FIRST_LAUNCH_RPM);

                    // Drive to score
                    motion.follower.followPath(secondLineEndToScore, PATH_VELOCITY_PERCENTAGE, true);
                    incrementPathState();
                }
                break;
            case 24:
                // Park
                if(!ballLifter.isLifting()){
                    // Stop the launcher
                    launcher.setVelocityRPM(0);

                    if (opmodeTimer.getElapsedTimeSeconds() >27) {
                        setPathState(-1);
                        break;
                    }

                    // Drive to park
                    motion.follower.followPath(scoreToPark, PATH_VELOCITY_PERCENTAGE, true);
                    incrementPathState();
                }
                break;
            case 25:
                // Need to actually wait for park
                if(!motion.follower.isBusy()) {
                    setPathState(-1);
                }
        }
    }
}