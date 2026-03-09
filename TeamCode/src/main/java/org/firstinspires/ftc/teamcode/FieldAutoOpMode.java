package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto: Field", group = "Robot")
@Disabled
public class FieldAutoOpMode extends AutoOpMode {
    private static final double FIRST_LAUNCH_RPM = 3800.00;
    private static final double SECOND_LAUNCH_RPM = 3900.00;
    private static final int SHOT_DELAY_MS = 50; // Ball settle time
    private double initialDelaySeconds = 0;
    private Pose startPose;
    private Pose initialScorePose;
    private Pose thirdLineStart;
    private Pose thirdLineEnd;
    private Pose jamStart;
    private Pose parkPose;


    private PathChain startToScorePath;
    private PathChain scoreToThirdLinePath;
    private PathChain thirdLineEndPath;
    private PathChain thirdLineEndToScore;
    private PathChain scoreToCornerJam;
    private PathChain cornerJam;
    private PathChain jamToScore;
    private PathChain scoreToPark;

    private boolean skipLine = false;

    private final double centerLineX = 72.0;
    private final double startPoseX = 10.5;
    private final double startPoseY = 8.5;
    private final double scorePoseX = 13.5;
    private final double scorePoseY = 22.5;
    private final double lineStartX = 31.0;
    private final double lineEndX = 52.0;
    private final double lineStartY = 33.5;
    private final double jamStartX = 72.0-10.0;
    private final double jamStartY = 20.0;
    private final double parkPoseX = 35.0;
    private final double parkPoseY = 12.5;

    public void setSkipLine() {
        skipLine = true;
    }

    public void setInitialDelaySeconds(double delay_ms){
        initialDelaySeconds = (delay_ms > 0) ? delay_ms : 0;
    }

    @Override
    public void init() {
        startPose = new Pose((color == COLOR.BLUE) ? centerLineX - startPoseX : centerLineX + startPoseX, startPoseY, Math.toRadians(90));
        initialScorePose = new Pose((color == COLOR.BLUE) ? centerLineX - scorePoseX : centerLineX + scorePoseX, scorePoseY, Math.toRadians((color == COLOR.BLUE) ? 113 : 67));
        thirdLineStart = new Pose((color == COLOR.BLUE) ? centerLineX - lineStartX : centerLineX + lineStartX, lineStartY, Math.toRadians(90));
        thirdLineEnd = new Pose((color == COLOR.BLUE) ? centerLineX - lineEndX : centerLineX + lineEndX, lineStartY+1.5, Math.toRadians(90));
        jamStart = new Pose((color == COLOR.BLUE) ? centerLineX - jamStartX : centerLineX + jamStartX, jamStartY, Math.toRadians((color == COLOR.BLUE) ? 135.0 : 45.0));
        parkPose = new Pose ((color == COLOR.BLUE) ? centerLineX - parkPoseX : centerLineX + parkPoseX, parkPoseY, Math.toRadians((color == COLOR.BLUE) ? 0 : 180));
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
        scoreToCornerJam = motion.follower.pathBuilder()
                .addPath(new BezierLine(initialScorePose, jamStart))
                .setLinearHeadingInterpolation(initialScorePose.getHeading(), jamStart.getHeading())
                .build();
        scoreToPark = motion.follower.pathBuilder()
                .addPath(new BezierLine(initialScorePose, parkPose))
                .setLinearHeadingInterpolation(initialScorePose.getHeading(), parkPose.getHeading())
                .build();

        // Corner jam!
        double jamAngle = Math.toRadians((color == COLOR.BLUE) ? 180 : 0);
        double jamNudgeAngle = Math.toRadians((color == COLOR.BLUE) ? 5 : -5);

        cornerJam = motion.follower.pathBuilder()
                //go in to get ball (45 deg angle)
                .addPath(new BezierLine(jamStart, jamStart.withY(jamStartY-2.0)))
                .setConstantHeadingInterpolation(jamStart.getHeading())
                //back up (45 deg angle)
                .addPath(new BezierLine(jamStart.withY(jamStartY-2.0), jamStart.withY(jamStartY-1.0)))
                .setConstantHeadingInterpolation(jamStart.getHeading())
                //go in to get ball (45 deg angle)
                .addPath(new BezierLine(jamStart.withY(jamStartY-1.0), jamStart.withY(jamStartY-4.0)))
                .setConstantHeadingInterpolation(jamStart.getHeading())
                //back up (45 deg angle)
                .addPath(new BezierLine(jamStart.withY(jamStartY-4.0), jamStart.withY(jamStartY-3.0)))
                .setConstantHeadingInterpolation(jamStart.getHeading())
                //go in to get ball (flat angle)
                .addPath(new BezierLine(jamStart.withY(jamStartY-3.0), jamStart.withY(jamStartY-2.0)))
                .setConstantHeadingInterpolation(jamAngle)
                // Nudge if stuck on ball) (flat+5 angle)
                .addPath(new BezierLine(jamStart.withY(jamStartY-2.0), jamStart.withY(jamStartY-2.1)))
                .setConstantHeadingInterpolation(jamStart.getHeading()+jamNudgeAngle)
                //go in to get ball (flat angle)
                .addPath(new BezierLine(jamStart.withY(jamStartY-2.1), jamStart.withY(jamStartY-7.0)))
                .setConstantHeadingInterpolation(jamAngle)
                // Nudge if stuck on ball) (flat angle)
                .addPath(new BezierLine(jamStart.withY(jamStartY-7.0), jamStart.withY(jamStartY-7.1)))
                .setConstantHeadingInterpolation(jamAngle)
                // Back to jam angle (45 degrees)
                .addPath(new BezierLine(jamStart.withY(jamStartY-7.1), jamStart.withY(jamStartY-7.0)))
                .setConstantHeadingInterpolation(jamStart.getHeading()+jamNudgeAngle)
                //go in to get ball (45 deg angle)
                .addPath(new BezierLine(jamStart.withY(jamStartY-7.0), jamStart.withY(jamStartY-8.0)))
                .setConstantHeadingInterpolation(jamStart.getHeading())
                .build();
        jamToScore = motion.follower.pathBuilder()
                .addPath(new BezierLine(motion.follower::getPose, initialScorePose))
                .setLinearHeadingInterpolation(jamStart.getHeading(), initialScorePose.getHeading())
                .build();

        setPathState(0);
    }

    private void delayedColorQueue(Sorter.BallColor color, int msDelay) {
        // Schedules a sorter rotation for the future
        // We try to launch the tasked color first, but will queue the other color if not found
        ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);
        Runnable task = () -> {
            if (color == Sorter.BallColor.Green) {
                if (!sorter.rotateGreenToLaunch())
                    sorter.rotatePurpleToLaunch();
            } else {
                if (!sorter.rotatePurpleToLaunch())
                    sorter.rotateGreenToLaunch();
            }
        };
        scheduler.schedule(task, msDelay, TimeUnit.MILLISECONDS);
    }

    private boolean firstTime = true;
    private double jamStartTime_s;
    private static final double MAX_JAM_TIME_S = 5.0;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(firstTime) {
                    // Follow first path for initial shots
                    motion.follower.followPath(startToScorePath, 0.7, true);

                    // Loaded purple left and back and green right when looking at robot
                    sorter.rotateClockwise(launchPattern.get(launchIndex++));

                    // Spin up launcher
                    launcher.setVelocityRPM(FIRST_LAUNCH_RPM);

                    // Done setting everything the first time
                    firstTime = false;
                }

                // Can we go to the next state?
                if (opmodeTimer.getElapsedTimeSeconds() > initialDelaySeconds)
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
            // Final 3 balls
            case 18:
            case 20:
            case 22:
                if (opmodeTimer.getElapsedTimeSeconds() > 28.5) {
                    motion.follower.breakFollowing();
                    setPathState(23);
                } else {
                    boolean ready = !motion.follower.isBusy() && sorter.isNotSpinning() && ballLifter.isReset();
                    //if(pathState == 1 || pathState == 10 )
                        ready  = ready && launcher.launchReady();
                    if(ready) {
                        // Stop the intakes
                        intake.leftIntakeStop();
                        intake.rightIntakeStop();
                        intake.frontIntakeStop();
                        sorter.leftAutoTurnOff();
                        sorter.rightAutoTurnOff();
                        sorter.frontAutoTurnOff();

                        // Launch (delay lets ball settle from rotation or driving)
                        if(pathState != 1) {
                            try {
                                Thread.sleep(SHOT_DELAY_MS);
                            } catch (InterruptedException ignore) {
                            }
                        }
                        if (pathState <= 5 || sorter.getBallCount()>0){
                            ballLifter.lift();
                        }
                        if(pathState == 5 && skipLine) {
                            pathState = 15;
                        }
                        else {
                            incrementPathState();
                        }
                    }
                }
                break;
            case 2:
            case 4:
                // Are we done lifting?
                if(ballLifter.isNotLifting()){
                    sorter.rotateClockwise(launchPattern.get(launchIndex++));
                    incrementPathState();
                }
                break;
            case 11:
            case 13:
            case 19:
            case 21:
                // Are we done lifting?
                if(ballLifter.isNotLifting()){
                    if(sorter.getBallCount() >= 1) {
                        if (colorPattern.get(launchIndex++) == Sorter.BallColor.Green) {
                            if (!sorter.rotateGreenToLaunch())
                                sorter.rotatePurpleToLaunch();
                        } else {
                            if (!sorter.rotatePurpleToLaunch())
                                sorter.rotateGreenToLaunch();
                        }
                    }
                    incrementPathState();
                }
                break;
            case 6:
                if (opmodeTimer.getElapsedTimeSeconds() >28) {
                    motion.follower.breakFollowing();
                    setPathState(23);
                }
                // Are we done lifting?
                else if(ballLifter.isNotLifting()){
                    // Slow down the launcher
                    launcher.setVelocityRPM(1500);

                    // Drive to pick up first line of balls
                    motion.follower.followPath(scoreToThirdLinePath, PATH_VELOCITY_PERCENTAGE, true);
                    incrementPathState();
                }
                break;
            case 7:
                // Pick up balls
                if(!motion.follower.isBusy()){
                    // Start front intake
                    if(color == COLOR.BLUE) {
                        intake.leftIntakeOn();
                        sorter.leftAutoTurnOn();
                    }
                    else{
                        intake.rightIntakeOn();
                        sorter.rightAutoTurnOn();
                    }

                    motion.follower.followPath(thirdLineEndPath, pickupPower, false);
                    incrementPathState();
                }
                break;
            case 8:
                // Drive to launch again
                if(!motion.follower.isBusy()){
                    // After end of line delay spinning to color for 1 second (finish intake)
                    if(sorter.getBallCount() >= 1) {
                        delayedColorQueue(colorPattern.get(launchIndex++), 500);
                    }

                    // Spin up launcher
                    launcher.setVelocityRPM(SECOND_LAUNCH_RPM);

                    // Drive to score
                    motion.follower.followPath(thirdLineEndToScore, 0.8, true);
                    incrementPathState();
                }
                break;
            case 9:
                if(!motion.follower.isBusy()){
                    // Intake off
                    if(color == COLOR.BLUE) {
                        intake.leftIntakeStop();
                        sorter.leftAutoTurnOff();
                    }else {
                        intake.rightIntakeStop();
                        sorter.rightAutoTurnOff();
                    }
                    incrementPathState();
                }
                break;
            case 15:
                if(ballLifter.isNotLifting()){
                    // Slow down the launcher
                    launcher.setVelocityRPM(1500);

                    // Drive to jam
                    motion.follower.followPath(scoreToCornerJam, PATH_VELOCITY_PERCENTAGE, false);
                    incrementPathState();
                }
                break;
            case 16:
                if(!motion.follower.isBusy()){
                    // Intake!
                    if(color == COLOR.BLUE) {
                        intake.leftIntakeOn();
                        sorter.leftAutoTurnOn();
                    }else {
                        intake.rightIntakeOn();
                        sorter.rightAutoTurnOn();
                    }

                    // Jam
                    jamStartTime_s = opmodeTimer.getElapsedTimeSeconds();
                    motion.follower.followPath(cornerJam, PATH_VELOCITY_PERCENTAGE, false);
                    incrementPathState();
                }
                break;
            case 17:
                // Not enough time to launch, go park
                if (opmodeTimer.getElapsedTimeSeconds() > 26) {
                    motion.follower.breakFollowing();
                    setPathState(23);
                }
                // Done jamming or jamming took to long (likely stuck), continue on
                else if(!motion.follower.isBusy() || opmodeTimer.getElapsedTimeSeconds() - jamStartTime_s > MAX_JAM_TIME_S){
                    // Jam stuck, stop following
                    if(motion.follower.isBusy())
                        motion.follower.breakFollowing();

                    // After end of line delay spinning to color for 1 second (finish intake)
                    if(sorter.getBallCount() >= 1) {
                        delayedColorQueue(colorPattern.get(launchIndex++), 500);
                    }

                    // Spin up launcher
                    launcher.setVelocityRPM(SECOND_LAUNCH_RPM);

                    // Drive to score
                    motion.follower.followPath(jamToScore, PATH_VELOCITY_PERCENTAGE, true);
                    incrementPathState();
                }
                break;
            case 23:
                // Park
                if(!motion.follower.isBusy() && ballLifter.isNotLifting()){
                    // Stop the launcher
                    launcher.setVelocityRPM(0);

                    // Stop intake
                    if(color == COLOR.BLUE) {
                        intake.leftIntakeStop();
                        sorter.leftAutoTurnOff();
                    }
                    else{
                        intake.rightIntakeStop();
                        sorter.rightAutoTurnOff();
                    }

                    // Drive to park
                    motion.follower.followPath(scoreToPark, PATH_VELOCITY_PERCENTAGE, true);
                    incrementPathState();
                }
                break;
            case 24:
                if(!motion.follower.isBusy()) {
                    setPathState(-1);
                }
        }
    }
}
