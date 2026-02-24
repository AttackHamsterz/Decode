package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Red Field End Early", group = "Auto: Red")
public class RedFieldAutoOpModeEndEarly extends FieldAutoOpMode {
    @Override public void init() {
        setup(COLOR.RED, POSITION.FIELD, true);
        super.init();
        setEndEarly();
    }
}