package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Red Board", group = "Auto: Red")
public class RedBoardAutoOpMode extends BoardAutoOpMode {
    @Override final public void init() {
        setup(COLOR.RED, POSITION.BOARD, true);
        super.init();
    }
}