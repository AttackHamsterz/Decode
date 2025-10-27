package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto: Board", group = "Robot")
@Disabled
public class BoardAutoOpMode extends AutoOpMode{
    private static final double FIRST_LAUNCH_RPM = 2500.00;
    private final Pose startPose = new Pose(16.08, 110.62, Math.toRadians(0));
    private final Pose initialScorePose = new Pose(27.78, 112.52, Math.toRadians(135));
    private final Pose firstLineStart = new Pose(42.0,89.0, Math.toRadians(180));
    private final Pose firstLineEnd = new Pose(23.08,89.0, Math.toRadians(180));
    private final Pose parkPose = new Pose( 38.0, 90.0, Math.toRadians(270.0));
    private final Pose secondLineStart = new Pose(42.0, 66.5, Math.toRadians(180.0));
    private final Pose secondLineEnd = new Pose(23.08, 66.5, Math.toRadians(180));

    private Path boardToScorePath;
    private PathChain scoreToFirstLinePath;
    private PathChain firstLineEndPath;
    private PathChain firstLineEndToScore;
    private PathChain scoreToPark;
    private PathChain scoreToSecondLine;
    private PathChain secondLineEndPath;
    private PathChain secondLineEndToScore;

    @Override
    public void buildPaths() {
        boardToScorePath = new Path(new BezierLine(startPose, initialScorePose));
        boardToScorePath.setLinearHeadingInterpolation(startPose.getHeading(), initialScorePose.getHeading());
        scoreToFirstLinePath = Motion.follower.pathBuilder()
                .addPath(new BezierLine(initialScorePose, firstLineStart))
                .setLinearHeadingInterpolation(initialScorePose.getHeading(), firstLineStart.getHeading())
                .build();
        firstLineEndPath = Motion.follower.pathBuilder()
                .addPath(new BezierLine(firstLineStart, firstLineEnd))
                .setLinearHeadingInterpolation(firstLineStart.getHeading(), firstLineEnd.getHeading())
                .build();
        firstLineEndToScore = Motion.follower.pathBuilder()
                .addPath(new BezierLine(firstLineEnd, initialScorePose))
                .setLinearHeadingInterpolation(firstLineEnd.getHeading(), initialScorePose.getHeading())
                .build();
        scoreToSecondLine = Motion.follower.pathBuilder()
                .addPath(new BezierLine(initialScorePose, secondLineStart))
                .setLinearHeadingInterpolation(initialScorePose.getHeading(), secondLineStart.getHeading())
                .build();
        secondLineEndPath = Motion.follower.pathBuilder()
                .addPath(new BezierLine(secondLineStart, secondLineEnd))
                .setLinearHeadingInterpolation(secondLineStart.getHeading(), secondLineEnd.getHeading())
                .build();
        secondLineEndToScore = Motion.follower.pathBuilder()
                .addPath(new BezierLine(secondLineEnd, initialScorePose))
                .setLinearHeadingInterpolation(secondLineEnd.getHeading(), initialScorePose.getHeading())
                .build();
        scoreToPark = Motion.follower.pathBuilder()
                .addPath(new BezierLine(initialScorePose, parkPose))
                .setLinearHeadingInterpolation(initialScorePose.getHeading(), parkPose.getHeading())
                .build();
        setPathState(0);
   }

    @Override public void init() {
        super.init();
        Motion.follower.setStartingPose(startPose);
        telemetry.addData("Auto Pose", startPose);
        telemetry.update();
        ballLifter.lift();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Follow first path for initial shots
                Motion.follower.followPath(boardToScorePath, true);

                // Adjust sorter to correct color - TODO
                sorter.rotateLeftToLaunch();

                // Spin up launcher
                launcher.setVelocityRPM(FIRST_LAUNCH_RPM);
                incrementPathState();
                break;
            case 1:
            case 3:
            case 5:
                // Done driving, sorter ready, launcher ready, lifter reset?
                if(!Motion.follower.isBusy() && !sorter.isSpinning() && launcher.launchReady() && ballLifter.isReset()) {
                    // Launch
                    try {
                        Thread.sleep(200);
                    } catch (InterruptedException e) {
                    }
                    ballLifter.lift();
                    incrementPathState();
                }
                break;
            case 2:
            case 4:
                // Are we done lifting?
                if(!ballLifter.isLifting()){
                    // Rotate sorter to next color - TODO
                    sorter.rotateRightToLaunch();
                    incrementPathState();
                }
                break;
            case 6:
                // Are we done lifting?
                if(!ballLifter.isLifting()){
                    // Slow down the launcher
                    launcher.setVelocityRPM(0);

                    // Start front intake


                    // Drive to pick up first line of balls
                    Motion.follower.followPath(scoreToFirstLinePath, true);
                    incrementPathState();
                }
                break;
            case 7:
                // Pick up balls
                if(!Motion.follower.isBusy()){
                    Motion.follower.followPath(firstLineEndPath, true);
                    incrementPathState();
                }
                break;

            case 8:
                // Drive to launch again
                if(!Motion.follower.isBusy()){
                    Motion.follower.followPath(firstLineEndToScore, true);
                    incrementPathState();
                }
                break;
            case 9:
                // Score balls
                if(!Motion.follower.isBusy()) {
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                    }
                    incrementPathState();
                }
                break;
            case 10:
                //Drive from score pose to second line start
                //if(!ballLifter.isLifting()){
                    Motion.follower.followPath(scoreToSecondLine, true);
                    incrementPathState();
                //}
                break;
            case 11:
                //Pick up balls
                if(!Motion.follower.isBusy()) {
                    Motion.follower.followPath(secondLineEndPath, true);
                    incrementPathState();
                }
                break;
            case 12:
                //Drive to score next round of balls
                if(!Motion.follower.isBusy()){
                    Motion.follower.followPath(secondLineEndToScore, true);
                    incrementPathState();
                }
                break;
            case 13:
                // Park
                if(!Motion.follower.isBusy()){
                    Motion.follower.followPath(scoreToPark, true);
                    incrementPathState();
                }
                break;
            default:
                if(!Motion.follower.isBusy()) {
                    launcher.setVelocityRPM(0);
                    setPathState(-1);
                }
        }
    }
}