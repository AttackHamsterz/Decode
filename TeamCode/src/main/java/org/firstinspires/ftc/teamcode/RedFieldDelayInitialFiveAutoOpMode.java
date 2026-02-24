package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Red Field 5s Delay", group= "Auto: Red")
public class RedFieldDelayInitialFiveAutoOpMode extends RedFieldAutoOpMode {
    @Override public void init() {
        setInitialDelaySeconds(5.0);
        super.init();
    }
}