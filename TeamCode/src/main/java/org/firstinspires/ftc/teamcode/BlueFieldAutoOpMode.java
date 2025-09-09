package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Blue Field", group = "Robot")
public class BlueFieldAutoOpMode extends FieldAutoOpMode {
    @Override final public void init() {
        setup(COLOR.BLUE, POSITION.FIELD, true);
        super.init();
    }
}