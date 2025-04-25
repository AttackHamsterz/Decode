package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Red Left", group = "Robot")
public class RedLeftAutoOpMode extends LeftAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        setup(COLOR.RED, SIDE.LEFT, true);
        super.runOpMode();
    }
}