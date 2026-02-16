package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@TeleOp(name = "OTOS Calibration", group = "Robot")
public class OtosCalOpMode extends OpMode {
    public Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.getPose();
        if(pose != null) {
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
            telemetry.addLine("Scale Factor is expected divided by measured");
            telemetry.addLine("Push forward 100 inches for linear scalar");
            telemetry.addLine("Spin 10 times and generate rotation factor");
            telemetry.update();
        }
    }
}
