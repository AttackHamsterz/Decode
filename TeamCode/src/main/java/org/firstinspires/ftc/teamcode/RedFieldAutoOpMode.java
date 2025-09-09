package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Red Right", group = "Robot")
public class RedFieldAutoOpMode extends FieldAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        setup(COLOR.RED, POSITION.FIELD, true);
        super.runOpMode();
    }
}