package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Blue Field", group = "Auto: Blue")
public class BlueFieldAutoOpMode extends FieldAutoOpMode {
    @Override public void init() {
        setup(COLOR.BLUE, POSITION.FIELD, true);
        super.init();
    }
}