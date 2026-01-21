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
    protected final double PICKUP_VELOCITY_PERCENTAGE = 0.18;
    private static final double FIRST_LAUNCH_RPM = 4250.00;
    private static final double SECOND_LAUNCH_RPM = 4250.00;
    private static final int SHOT_DELAY_MS = 50; // Ball settle time
    private static final int LINE_END_DELAY_MS = 1000;
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
        final double startPoseX = 10.5;
        final double scorePoseX = 13.5;
        final double lineStartX = 30.0;
        final double lineEndX = 55;
        final double parkPoseX = 35.0;

        startPose = new Pose((color == COLOR.BLUE) ? centerLineX - startPoseX : centerLineX + startPoseX, 8.5, Math.toRadians(90));
        initialScorePose = new Pose((color == COLOR.BLUE) ? centerLineX - scorePoseX : centerLineX + scorePoseX, 20.5, Math.toRadians((color == COLOR.BLUE) ? 112 : 68));
        thirdLineStart = new Pose((color == COLOR.BLUE) ? centerLineX - lineStartX : centerLineX + lineStartX, 35, Math.toRadians((color == COLOR.BLUE) ? 180 : 0));
        thirdLineEnd = new Pose((color == COLOR.BLUE) ? centerLineX - lineEndX :centerLineX + lineEndX, 35, Math.toRadians((color == COLOR.BLUE) ? 180 : 0));
        parkPose = new Pose ((color == COLOR.BLUE) ? centerLineX - parkPoseX : centerLineX + parkPoseX, 12.5, Math.toRadians((color == COLOR.BLUE) ? 0 : 180));
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

        public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Follow first path for initial shots
                motion.follower.followPath(startToScorePath, 0.7, true);

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
                if(!motion.follower.isBusy() && sorter.isNotSpinning() && launcher.launchReady() && ballLifter.isReset()) {
                    // Stop the front intake (needed for 9 and 17)
                    if(pathState==9) {
                        intake.frontIntakeStop();
                        sorter.frontAutoTurnOff();
                    }

                    // Launch (delay lets ball settle from rotation)
                    try {
                        Thread.sleep(SHOT_DELAY_MS);
                    } catch (InterruptedException ignore) {
                    }
                    if (!(pathState>5 && sorter.getBallCount()<1)){
                        ballLifter.lift();
                    }
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
                    intake.frontIntakeOn();
                    sorter.frontAutoTurnOn();

                    motion.follower.followPath(thirdLineEndPath, pickupPower, false);
                    incrementPathState();
                }
                break;
            case 8:
                // Drive to launch again
                if(!motion.follower.isBusy()){
                    // After end of line delay spinning to color for 1 second (finish intake)
                    delayedColorQueue(colorPattern.get(launchIndex++), 500);

                    // Spin up launcher
                    launcher.setVelocityRPM(SECOND_LAUNCH_RPM);

                    // Drive to score
                    motion.follower.followPath(thirdLineEndToScore, 0.7, true);
                    incrementPathState();
                }
                break;
            case 9:
                if(!motion.follower.isBusy()){
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException ignore) {}

                    intake.frontIntakeStop();
                    sorter.frontAutoTurnOff();

                    // After end of line delay spinning to color for 1 second (finish intake)
                    delayedColorQueue(colorPattern.get(launchIndex++), 0);
                    incrementPathState();
                }
                break;
            case 15:
                // Park
                if(!ballLifter.isLifting()){
                    // Stop the launcher
                    launcher.setVelocityRPM(0);

                    // Drive to park
                    motion.follower.followPath(scoreToPark, 0.8, true);
                    incrementPathState();
                }
                break;
            case 16:
                if(!motion.follower.isBusy()) {
                    setPathState(-1);
                }
        }
    }
}
