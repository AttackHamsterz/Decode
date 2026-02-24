package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Red Field", group = "Auto: Red")
public class RedFieldAutoOpMode extends FieldAutoOpMode {
    @Override public void init() {
        setup(COLOR.RED, POSITION.FIELD, true);
        super.init();
    }
}