package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp: Decode", group="Robot")
@Disabled
public class DecodeOpMode extends StandardSetupOpMode {
    @Override public void loop() {
        // Update telemetry
        motion.getTelemetry(telemetry);
        sorter.getTelemetry(telemetry);
        telemetry.update();

        // Short sleep to keep this loop from saturating
        try {
            Thread.sleep(RobotPart.LOOP_PAUSE_MS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
