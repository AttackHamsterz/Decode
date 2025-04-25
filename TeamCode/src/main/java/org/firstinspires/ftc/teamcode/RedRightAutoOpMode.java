package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Red Right", group = "Robot")
public class RedRightAutoOpMode extends RightAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        setup(COLOR.RED, SIDE.RIGHT, true);
        super.runOpMode();
    }
}