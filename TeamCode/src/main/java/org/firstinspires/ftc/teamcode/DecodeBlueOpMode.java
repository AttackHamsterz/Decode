package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp: Decode Blue", group="Robot")
public class DecodeBlueOpMode extends DecodeOpMode {
    @Override public void init() {
        setup(COLOR.BLUE, POSITION.BOARD, false);
        super.init();
    }
}
