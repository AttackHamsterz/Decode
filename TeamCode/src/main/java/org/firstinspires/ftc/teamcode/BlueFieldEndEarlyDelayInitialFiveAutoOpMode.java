package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Blue Field Skip Row3 5s Delay", group= "Auto: Blue")
public class BlueFieldEndEarlyDelayInitialFiveAutoOpMode extends BlueFieldAutoOpModeEndEarly {
    @Override public void init() {
        setInitialDelaySeconds(5.0);
        super.init();
    }
}

