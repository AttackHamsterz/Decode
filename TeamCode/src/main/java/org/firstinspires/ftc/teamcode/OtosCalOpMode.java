package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "OTOS Calibration", group = "Robot")
public class OtosCalOpMode extends OpMode {
    public Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void loop() {
        Pose pose = follower.getPose();
        if(pose != null) {
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
            telemetry.addLine("Scale Factor is expected over measured");
            telemetry.addLine("Push forward 100 inches for linear scalar");
            telemetry.addLine("Spin 10 times and generate rotation factor");
            telemetry.update();
        }
    }
}
