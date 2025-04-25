package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp: Decode Blue", group="Robot")
public class DecodeBlueOpMode extends DecodeOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Call the parent runOpMode
        setup(COLOR.BLUE, SIDE.LEFT, false);
        super.runOpMode();
    }
}
