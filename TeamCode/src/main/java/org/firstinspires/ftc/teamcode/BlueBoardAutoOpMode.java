package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Blue Board", group = "Robot")
public class BlueBoardAutoOpMode extends BoardAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        setup(COLOR.BLUE, POSITION.BOARD, true);
        super.runOpMode();
    }
}