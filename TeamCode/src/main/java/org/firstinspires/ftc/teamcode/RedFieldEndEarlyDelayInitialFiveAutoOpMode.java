package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Red Field End Early 5s Delay", group= "Auto: Red")
public class RedFieldEndEarlyDelayInitialFiveAutoOpMode extends RedFieldAutoOpModeEndEarly {
    @Override public void init() {
        setInitialDelaySeconds(5.0);
        super.init();
    }
}