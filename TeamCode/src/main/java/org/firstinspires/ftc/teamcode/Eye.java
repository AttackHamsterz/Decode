package org.firstinspires.ftc.teamcode;

import static java.lang.Math.tan;

import androidx.annotation.NonNull;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/**
 * An eye class that uses a Limelight for various modes of play.
 * Each mode will use a specific pipeline.
 */
public class Eye extends RobotPart<EyeMetric>{
    private static final double CLOSE_ENOUGH_DEGREES = 8.0;

    public enum Mode{
        AUTO_START(0),      // Locate the autonomous decision token
        AIM_POINT(1),       // Locate the aim point for placing or throwing
        FIELD_LOCALIZE(2),  // Locate fiducials markers for field localization
        NONE(3);            // No mode, do nothing

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
    private ColorOrder colorOrder;
    private int fiducialId;
    private double currentDegrees;
    private double shotD;
    private double deltaRPM;

    private PIDFController aimController;

    public Eye(StandardSetupOpMode ssom){
        this.ssom = ssom;
        this.mode = Mode.AUTO_START;

        // Setup limelight
        limelight = ssom.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(150); // This sets how often we ask Limelight for data (100 times per second)
        //Set up PIDF aim controller
        aimController = new PIDFController(new PIDFCoefficients(0.015, 0.0, 2.0, 0.0));
        aimController.setTargetPosition(0.0);
    }

    @Override
    public void run() {
        // Sanity
        if(limelight == null){
            ssom.telemetry.addLine("No Limelight!");
            ssom.telemetry.update();
            return;
        }

        // This tells Limelight to start looking!
        limelight.start();

        // This loop will continue until game end
        setRunning();
        boolean pressed = false;
        boolean g2pressed = false;
        while (running) {
            // Get a result if we're in a mode that needs one
            if (mode == Mode.AIM_POINT) {
                resultInUse = limelight.getLatestResult();
                List<LLResultTypes.FiducialResult> fiducials = resultInUse.getFiducialResults();
                for(LLResultTypes.FiducialResult fiducial : fiducials)
                {
                    if(fiducial.getFiducialId() == 20 && ssom.color == StandardSetupOpMode.COLOR.BLUE){
                        //launcher speed
                        Position pos = fiducial.getRobotPoseTargetSpace().getPosition();
                        fiducialId = 20;
                        shotD = Math.sqrt(pos.x*pos.x+pos.y*pos.y+pos.z*pos.z);

                        //auto aiming
                        currentDegrees = fiducial.getTargetXDegrees();
                        aimController.updateError(currentDegrees);
                        ssom.motion.setTurn(aimController.run());

                        break;
                    }
                    else if (fiducial.getFiducialId() == 24 && ssom.color == StandardSetupOpMode.COLOR.RED){
                        //launcher speed
                        Position pos = fiducial.getRobotPoseTargetSpace().getPosition();
                        fiducialId = 24;
                        shotD = Math.sqrt(pos.x*pos.x+pos.y*pos.y+pos.z*pos.z);

                        //auto aiming
                        currentDegrees = fiducial.getTargetXDegrees();
                        aimController.updateError(currentDegrees);
                        ssom.motion.setTurn(aimController.run());
                        break;
                    }
                    else{
                        currentDegrees = 180.0;
                        fiducialId = -1;
                        shotD = 0;
                        aimController.reset();
                        ssom.motion.setTurn(0);
                    }
                }
                ssom.launcher.setRPMFromDistance(shotD,deltaRPM);
            }
            if (mode == Mode.AUTO_START) {
                /*
                resultInUse = limelight.getLatestResult();
                List<LLResultTypes.FiducialResult> fiducials = resultInUse.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int obeliskColorId = fiducial.getFiducialId();

                    if ( obeliskColorId >= 21 && obeliskColorId <= 23 ){
                        colorOrder = Eye.ColorOrder.fromId(obeliskColorId);
                        break;
                    }
                }
                 */
            }
            if(!ssom.gamepadBuffer.ignoreGamepad) {
                if (!pressed && ssom.gamepadBuffer.g1RightBumper) {
                    enableAimMode();
                    pressed = true;
                }
                else if (pressed && !ssom.gamepadBuffer.g1RightBumper) {
                    disableAimMode();
                    pressed = false;
                }
                if(!g2pressed && ssom.gamepadBuffer.g2DpadUp){
                    deltaRPM += 20;
                    g2pressed = true;
                }
                else if(!g2pressed && ssom.gamepadBuffer.g2DpadDown){
                    deltaRPM -= 20;
                    g2pressed = true;
                }
                if(!ssom.gamepadBuffer.g2DpadDown && !ssom.gamepadBuffer.g2DpadUp){
                    g2pressed = false;
                }
            }

            // Short sleep if we're not aiming (to keep this loop from saturating)
            if(mode != Mode.AIM_POINT)
                sleep();
        }

        // Cleanup
        limelight.stop();
    }

    public void enableAimMode(){
        aimController.reset();
        setMode(Mode.AIM_POINT);
    }

    public void disableAimMode(){
        setMode(Mode.NONE);
        ssom.launcher.setRPMFromDistance(0, 0);
        ssom.motion.setTurn(0);
    }

    public boolean linedUp(){
        return Math.abs(currentDegrees) < CLOSE_ENOUGH_DEGREES;
    }

    public enum ColorOrder{
        GPP (21, new String[] {"Green", "Purple", "Purple"}),
        PGP (22,new String[] {"Purple", "Green", "Purple"}),
        PPG (23, new String[] {"Purple", "Purple", "Green"});

        private final String[] colors;
        private final int id;

        ColorOrder(int id, String[] colors) {
            this.id = id;
            this.colors = colors;
        }
        public int getId() {
            return id;
        }

        public String[] getColors() {
            return colors;
        }

        public static ColorOrder fromId(int obeliskColorId) {
            for (ColorOrder order : values()) {
                if (order.id == obeliskColorId) {
                    return order;
                }
            }
            throw new IllegalArgumentException("Invalid obeliskColorId" + obeliskColorId);
        }
    }

    public int getFiducialID() {
        resultInUse = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = resultInUse.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {

            int obeliskColorId = fiducial.getFiducialId();
            if ( obeliskColorId >= 21 && obeliskColorId <= 23 ) {
                return obeliskColorId;
            }
        }
        return -1;
    }

    public ColorOrder getColorOrder() {
        return colorOrder;
    }

    public ColorOrder getColorOrder(int id) {
        return colorOrder = Eye.ColorOrder.fromId(id);
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

    public double getShotDistance(){
        return shotD;
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        // Depending on the mode, the telemetry can change
        if((DEBUG & 4) != 0) {
            telemetry.addData("Mode", mode);
            telemetry.addData("Fiducial", fiducialId);
            telemetry.addData("Shot Distance", shotD);
        }
    }

}
