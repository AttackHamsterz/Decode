package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto: Field", group = "Robot")
@Disabled
public class FieldAutoOpMode extends AutoOpMode{
    private static final double FIRST_LAUNCH_RPM = 3200.00;

    private Pose startPose;
    private Pose initialScorePose;
    @Override public void init() {

        final double centerLineX = 72.0;
        final double startPoseX = 8.0;
        startPose = new Pose((color == COLOR.BLUE) ? centerLineX - startPoseX : centerLineX + startPoseX, 8.5, Math.toRadians(90));
        initialScorePose = new Pose((color == COLOR.BLUE) ? centerLineX - startPoseX : centerLineX + startPoseX,80, Math.toRadians(90));
    }
    }
