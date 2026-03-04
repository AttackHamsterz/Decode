package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp: Decode", group="Robot")
@Disabled
public class DecodeOpMode extends StandardSetupOpMode {
    @Override public void init() {
        super.init();
        // If we're blue team we intake on the right side more often so we favor clockwise
        // turns to keep balls moving toward the launcher and keep the front open for intake
        // If we're red team we intake more on the left side so we favor counter clockwise
        // turns to keep balls moving toward the launcher
        sorter.favorClockwise(color == StandardSetupOpMode.COLOR.BLUE);
    }
    @Override public void loop() {
        // Update gamepad for multiple thread use (loop is called at the optimal timing)
        if(!ignoreGamepad)
            gamepadBuffer.update(gamepad1, gamepad2);

        // Update telemetry
        super.partList.forEach(part -> part.getTelemetry(telemetry));
        if(super.partList.size()>0) telemetry.update();
    }
}
