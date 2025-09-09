package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Red Field", group = "Robot")
public class RedFieldAutoOpMode extends FieldAutoOpMode {
    @Override final public void init() {
        setup(COLOR.RED, POSITION.FIELD, true);
        super.init();
    }
}