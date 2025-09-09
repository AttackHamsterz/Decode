package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Blue Right", group = "Robot")
public class BlueFieldAutoOpMode extends FieldAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        setup(COLOR.BLUE, POSITION.FIELD, true);
        super.runOpMode();
    }
}