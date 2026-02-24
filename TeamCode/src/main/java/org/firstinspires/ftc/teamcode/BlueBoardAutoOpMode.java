package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Blue Board", group = "Auto: Blue")
public class    BlueBoardAutoOpMode extends BoardAutoOpMode {
    @Override final public void init() {
        setup(COLOR.BLUE, POSITION.BOARD, true);
        super.init();
    }
}