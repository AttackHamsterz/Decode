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
    private final Pose firstLineStart = new Pose(42.0,90.0, Math.toRadians(180));
    private final Pose firstLineEnd = new Pose(23.08,90.0, Math.toRadians(180));

    private Path boardToScorePath;
    private PathChain scoreToFirstLinePath;
    private PathChain firstLineEndPath;
    private PathChain firstLineEndToScore;

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
                        Thread.sleep(500);
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
                    Motion.follower.followPath(scoreToFirstLinePath);
                    incrementPathState();
                }
                break;
            case 7:
            case 10:
                // Pick up balls
                if(!Motion.follower.isBusy())
                {
                    Motion.follower.followPath(firstLineEndPath);
                    incrementPathState();
                }
                break;

            case 8:
            case 11:
                // Drive to launch again
                if(!Motion.follower.isBusy())
                {
                    Motion.follower.followPath(firstLineEndToScore, true);
                    incrementPathState();
                }
                break;
            case 9:
                // Drive to pick up second line of balls
                incrementPathState();
                break;
            case 12:
                // Park
                incrementPathState();
                break;
            default:
                launcher.setVelocityRPM(0);
                setPathState(-1);
        }
    }
}