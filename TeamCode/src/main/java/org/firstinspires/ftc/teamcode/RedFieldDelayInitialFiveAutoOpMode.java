package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Red Five Second Initial Delay", group= "Robot")
public class RedFieldDelayInitialFiveAutoOpMode extends RedFieldAutoOpMode {
    private static final int PARTNER_INITIAL_DELAY_MS = 5000; // Partner ball settle time

    @Override final public void initialFieldAutoDelay() {
        try {
            Thread.sleep(PARTNER_INITIAL_DELAY_MS);
        } catch (InterruptedException ignore) {
        }
    }
}