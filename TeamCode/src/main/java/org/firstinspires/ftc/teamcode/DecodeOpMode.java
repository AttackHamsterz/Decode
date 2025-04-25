package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp: Decode", group="Robot")
@Disabled
public class DecodeOpMode extends StandardSetupOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Call the parent runOpMode
        super.runOpMode();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Debugging
            //telemetry.addData("Status", "Run Time: " + runtime);
            //motion.debugTelemetry(telemetry);
            //telemetry.update();

            // Short sleep to keep this loop from saturating
            sleep(RobotPart.LOOP_PAUSE_MS);
        }

        // Interrupt threads and wait for completion
        waitForCompletion();
    }
}


