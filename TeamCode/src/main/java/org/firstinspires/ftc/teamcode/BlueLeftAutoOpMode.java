package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Blue Left", group = "Robot")
public class BlueLeftAutoOpMode extends LeftAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        setup(COLOR.BLUE, SIDE.LEFT, true);
        super.runOpMode();
    }
}