package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Red Field 3s Delay", group= "Auto: Red")
public class RedFieldDelayInitialThreeAutoOpMode extends RedFieldAutoOpMode {
    @Override public void init() {
        setInitialDelaySeconds(3.0);
        super.init();
    }
}