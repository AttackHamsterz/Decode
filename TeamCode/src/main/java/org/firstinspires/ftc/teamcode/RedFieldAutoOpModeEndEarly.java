package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Red Field End Early", group = "Robot")
public class RedFieldAutoOpModeEndEarly extends FieldAutoOpMode {
    @Override final public void init() {
        setup(COLOR.RED, POSITION.FIELD, true);
        super.init();
        setEndEarly();
    }
}