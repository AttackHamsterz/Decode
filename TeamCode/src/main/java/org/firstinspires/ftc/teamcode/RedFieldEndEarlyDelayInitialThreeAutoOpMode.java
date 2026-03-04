package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Red Field Skip Row3 3s Delay", group= "Auto: Red")
public class RedFieldEndEarlyDelayInitialThreeAutoOpMode extends RedFieldAutoOpModeEndEarly {
    @Override public void init() {
        setInitialDelaySeconds(3.0);
        super.init();
    }
}