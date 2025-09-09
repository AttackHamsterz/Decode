package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Red Board", group = "Robot")
public class RedBoardAutoOpMode extends BoardAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        setup(COLOR.RED, POSITION.BOARD, true);
        super.runOpMode();
    }
}