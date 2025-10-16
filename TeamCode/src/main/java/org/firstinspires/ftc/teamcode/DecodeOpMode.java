package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp: Decode", group="Robot")
@Disabled
public class DecodeOpMode extends StandardSetupOpMode {
    @Override public void loop() {
        // Update gamepad for multiple thread use (loop is called at the optimal timing)
        if(!ignoreGamepad)
            gamepadBuffer.update(gamepad1, gamepad2);

        // Update telemetry
        super.partList.forEach(part -> part.getTelemetry(telemetry));
        if(super.partList.size()>0) telemetry.update();
    }
}
