package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * An eye class that uses a Limelight for various modes of play.
 * Each mode will use a specific pipeline.
 */
public class Eye extends RobotPart<EyeMetric>{

    public enum Mode{
        AUTO_START(0),      // Locate the autonomous decision token
        AIM_POINT(1),       // Locate the aim point for placing or throwing
        FIELD_LOCALIZE(2),  // Locate fiducials markers for field localization
        TARGET_LOCALIZE(3), // Locate fiducials for scoring target
        NONE(4);            // No mode, do nothing

        private final int value;
        Mode(int value){
            this.value = value;
        }
        @NonNull
        public String toString()
        {
            String output;
            switch(value){
                case 0:
                    output = "Autonomous Mode";
                    break;
                case 1:
                    output = "Aim Mode";
                    break;
                case 2:
                    output = "Field Mode";
                    break;
                case 3:
                    output = "Target Mode";
                    break;
                case 4:
                default:
                    output = "No Mode";
                    break;
            };
            return output;
        }
    };
    private final Limelight3A limelight;
    private Eye.Mode mode;
    private LLResult resultInUse;

    public Eye(StandardSetupOpMode ssom){
        this.ssom = ssom;
        this.mode = Mode.AUTO_START;

        // Setup limelight
        limelight = ssom.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
    }

    @Override
    public void run() {
        // Sanity
        if(limelight == null) return;

        // This tells Limelight to start looking!
        limelight.start();

        // This loop will continue until game end
        while (!isInterrupted()) {
            // Get a result if we're in a mode that needs one
            if(mode != Mode.NONE){
                resultInUse = limelight.getLatestResult();
                int pi = resultInUse.getPipelineIndex();
            }
            if (mode == Mode.AIM_POINT) {
                resultInUse = limelight.getLatestResult();
                double tx = resultInUse.getTx();
                double xrange = Range.clip(tx/25.0, -1.0, 1.0);
                double Kp = xrange * 0.1;
                ssom.motion.spin(Kp);
            }
            if (ssom.gamepadBuffer.g1LeftBumper) {
                setMode(Mode.AIM_POINT);
            }
            // Short sleep to keep this loop from saturating
            sleep();
        }
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
    }

    public void setMode(Mode newMode) {
        mode = newMode;
    }

    @Override
    protected void setTo(EyeMetric metric) {

    }

    @Override
    public void safeHold() {

    }

    @Override
    protected boolean closeEnough(EyeMetric metric) {
        // Depending on the mode, the eye metric can change
        return false;
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        // Depending on the mode, the telemetry can change
        if (resultInUse != null && resultInUse.isValid()) {
            double tx = resultInUse.getTx(); // How far left or right the target is
            double ty = resultInUse.getTy(); // How far up or down the target is
            double ta = resultInUse.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Eye Mode", mode);
            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }
}
