package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Blue Field 5s Delay", group= "Auto: Blue")
public class BlueFieldDelayInitialFiveAutoOpMode extends BlueFieldAutoOpMode {
    @Override public void init() {
        setInitialDelaySeconds(5.0);
        super.init();
    }
}
