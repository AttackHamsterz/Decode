package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto: Board", group = "Robot")
@Disabled
public class BoardAutoOpMode extends AutoOpMode{

    private final Pose startPose = new Pose(16.08, 110.62, Math.toRadians(0));
    private final Pose initialScorePose = new Pose(27.78, 112.52, Math.toRadians(135));

    private Path scorePath;

    @Override
    public void buildPaths() {
        scorePath = new Path(new BezierLine(startPose, initialScorePose));
        scorePath.setLinearHeadingInterpolation(startPose.getHeading(), initialScorePose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Begin the autonomous
                motion.follower.followPath(scorePath);
                setPathState(1);
                break;
            case 1:

                if(!motion.follower.isBusy()) {

                    setPathState(-1);
                }
                break;

        }

    }
    }