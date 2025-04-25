package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Blue Right", group = "Robot")
public class BlueRightAutoOpMode extends RightAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        setup(COLOR.BLUE, SIDE.RIGHT, true);
        super.runOpMode();
    }
}