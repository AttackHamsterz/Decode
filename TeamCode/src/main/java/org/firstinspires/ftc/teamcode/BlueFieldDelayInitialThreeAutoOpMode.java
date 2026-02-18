package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Three Second Initial Delay", group= "Robot")
public class BlueFieldDelayInitialThreeAutoOpMode extends BlueFieldAutoOpMode {
    private static final int PARTNER_INITIAL_DELAY_MS = 3000; // Partner ball settle time

    @Override final public void initialFieldAutoDelay() {
        try {
            Thread.sleep(PARTNER_INITIAL_DELAY_MS);
        } catch (InterruptedException ignore) {
        }
    }
}