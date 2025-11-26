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

    private static final double FIRST_LAUNCH_RPM = 2500.00; // Launcher speed
    private static final int SHOT_DELAY_MS = 50;            // Ball settle time
    private static final int LINE_END_DELAY_MS = 1000;      // Wait after line to rotate to color

    private Pose startPose;
    private Pose initialScorePose;
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
        final double midLinePoseX = 25.0;
        final double midLinePoseY = 100.0;
        final double lineStartX = 25.0;
        final double firstLineStartY = 82.0;
        final double secondLineStartY = 58.5;
        final double lineEndX = 50.0;
        final double parkX = 49.0;
        final double parkY = 67.5;

        startPose = new Pose((color == COLOR.BLUE) ? centerLineX-startPoseX :centerLineX+startPoseX, startPoseY, Math.toRadians((color == COLOR.BLUE) ? 0 : 180));
        initialScorePose = new Pose((color == COLOR.BLUE) ? centerLineX-initialScorePoseX :centerLineX+initialScorePoseX, initialScorePoseY, Math.toRadians((color == COLOR.BLUE) ? 135 : 45));
        midLinePose = new Pose ((color == COLOR.BLUE) ? centerLineX-midLinePoseX :centerLineX+midLinePoseX, midLinePoseY, Math.toRadians((color == COLOR.BLUE) ? 180 : 0));
        firstLineStart = new Pose((color == COLOR.BLUE) ? centerLineX-lineStartX :centerLineX+lineStartX,firstLineStartY, Math.toRadians((color == COLOR.BLUE) ? 180 : 0));
        firstLineEnd = new Pose((color == COLOR.BLUE) ? centerLineX-lineEndX :centerLineX+lineEndX,firstLineStartY, Math.toRadians((color == COLOR.BLUE) ? 180 : 0));
        parkPose = new Pose( (color == COLOR.BLUE) ? centerLineX-parkX :centerLineX+parkX, parkY, Math.toRadians((color == COLOR.BLUE) ? 0 : 180));
        secondLineStart = new Pose((color == COLOR.BLUE) ? centerLineX-lineStartX :centerLineX+lineStartX, secondLineStartY, Math.toRadians((color == COLOR.BLUE) ? 180 : 0));
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
                .addPath(new BezierLine(secondLineEnd, initialScorePose))
                .setLinearHeadingInterpolation(secondLineEnd.getHeading(), initialScorePose.getHeading())
                .build();
        scoreToPark = motion.follower.pathBuilder()
                .addPath(new BezierLine(initialScorePose, parkPose))
                .setLinearHeadingInterpolation(initialScorePose.getHeading(), parkPose.getHeading())
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
            case 9:
            case 11:
            case 13:
            // Third 3 balls
            case 17:
            case 19:
            case 21:
                // Done driving, sorter ready, launcher ready, lifter reset?
                if(!motion.follower.isBusy() && !sorter.isSpinning() && launcher.launchReady() && ballLifter.isReset()) {
                    // Stop the front intake (needed for 9 and 17)
                    if(pathState==9 || pathState == 17) {
                        intake.frontIntakeStop();
                        sorter.autoTurnOff();
                    }

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
            case 10:
            case 12:
            case 18:
            case 20:
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
                    motion.follower.followPath(firstLineEndPath, PICKUP_VELOCITY_PERCENTAGE, false);
                    incrementPathState();
                }
                break;
            case 8:
                // Drive to launch again
                if(!motion.follower.isBusy()){
                    // After end of line delay spinning to color for 1 second (finish intake)
                    delayedColorQueue(colorPattern.get(launchIndex++), LINE_END_DELAY_MS);

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

                    // Drive to pick up first line of balls
                    motion.follower.followPath(scoreToSecondLine, PATH_VELOCITY_PERCENTAGE, false);
                    incrementPathState();
                }
                break;
            case 15:
                //Pick up balls
                if(!motion.follower.isBusy()) {
                    // Start front intake
                    intake.frontIntakeOn();
                    sorter.autoTurnOn();

                    motion.follower.followPath(secondLineEndPath, PICKUP_VELOCITY_PERCENTAGE, false);
                    incrementPathState();
                }
                break;
            case 16:
                //Drive to score next round of balls
                if(!motion.follower.isBusy()){
                    // After end of line delay spinning to color for 1 second (finish intake)
                    delayedColorQueue(colorPattern.get(launchIndex++), LINE_END_DELAY_MS);

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
                // Need to actually wait for park
                if(!motion.follower.isBusy()) {
                    setPathState(-1);
                }
        }
    }
}