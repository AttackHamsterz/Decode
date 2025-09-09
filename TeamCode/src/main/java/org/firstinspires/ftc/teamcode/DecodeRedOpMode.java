package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp: Decode Red", group="Robot")
public class DecodeRedOpMode extends DecodeOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Call the parent runOpMode
        setup(COLOR.RED, POSITION.BOARD, false);
        super.runOpMode();
    }
}
