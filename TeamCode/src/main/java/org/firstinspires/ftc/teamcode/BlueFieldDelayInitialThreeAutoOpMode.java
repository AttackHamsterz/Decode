package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Blue Field 3s Delay", group= "Auto: Blue")
public class BlueFieldDelayInitialThreeAutoOpMode extends BlueFieldAutoOpMode {
    @Override public void init() {
        setInitialDelaySeconds(3.0);
        super.init();
    }
}