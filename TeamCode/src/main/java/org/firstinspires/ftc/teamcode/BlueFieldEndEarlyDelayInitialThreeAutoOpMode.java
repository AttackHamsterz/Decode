package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Blue Field Skip Row3 3s Delay", group= "Auto: Blue")
public class BlueFieldEndEarlyDelayInitialThreeAutoOpMode extends BlueFieldAutoOpModeEndEarly {
    @Override public void init() {
        setInitialDelaySeconds(3.0);
        super.init();
    }
}

