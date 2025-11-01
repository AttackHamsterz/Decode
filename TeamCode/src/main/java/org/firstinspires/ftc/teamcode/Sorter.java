package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sorter extends RobotPart<SorterMetric>{
    private static final double PPM = 1425.1;
    private static final double HALF_TURN = PPM / 2.0;
    private static final double QUARTER_TURN = PPM / 4.0;
    private static final int CLOSE_ENOUGH_TICKS = 14;
    private static final double HOLD_POWER = 0.1;
    private static final double TAIL_ROTATE_TIME_MS = 100;
    private static final double QUARTER_TURN_POWER = 1.0;
    private static final double HALF_TURN_POWER = 1.0;

    public enum BallColor{
        None(0),
        Green(1),
        Purple(2);

        private final String color;
        private final int id;
        private double distance;
        private float[] hsv;
        private long time;

        private static final double MIN_DIST_CM = 3.0;
        private static final float PURPLE_HUE_MIN = 165;
        private static final float PURPLE_HUE_MAX = 240;
        private static final float GREEN_HUE_MIN = 150;
        private static final float GREEN_HUE_MAX = 165;
        private static final float SATURATION_MIN = 0.45f;
        private static final float VALUE_MIN = 0.18f;

        BallColor(int id) {
            if(id < 1 || id > 2) id = 0;
            this.id = id;
            this.color = (id == 1) ? "Green" : (id == 2) ? "Purple" : "None";
            this.hsv = new float[3];
            this.distance = 0;
            this.time = 0;
        }

        public void setHSV(float[] hsv){
            if(hsv.length > 2) {
                this.hsv[0] = hsv[0];
                this.hsv[1] = hsv[1];
                this.hsv[2] = hsv[2];
            }
        }

        public void setDistance(double distance){
            this.distance = distance;
        }

        public void setTime(long time){
            this.time = time;
        }

        public int getId() {
            return id;
        }

        public long getTime(){
            return time;
        }

        public double getDistance(){
            return distance;
        }

        @NonNull
        public String toString() {
            return color;
        }

        public static BallColor fromSensor(RevColorSensorV3 sensor) {
            BallColor ballColor = BallColor.None;
            float[] hsv = new float[3];
            long time = System.currentTimeMillis();
            double distance = 0;

            // Sanity
            if(sensor != null){
                // Get distance (if too far, then no color)
                distance = sensor.getDistance(DistanceUnit.CM);
                if(distance <= MIN_DIST_CM) {
                    // Get normalized RGB (adjust gain at each competition)
                    NormalizedRGBA colors = sensor.getNormalizedColors();

                    // Convert to HSV
                    Color.RGBToHSV(
                            (int) (colors.red * 255),
                            (int) (colors.green * 255),
                            (int) (colors.blue * 255),
                            hsv
                    );

                    // Compare to thresholds to generate correct Ball Color
                    if (hsv[1] >= SATURATION_MIN && hsv[2] >= VALUE_MIN) {
                        if (hsv[0] >= PURPLE_HUE_MIN && hsv[0] <= PURPLE_HUE_MAX)
                            ballColor = BallColor.Purple;
                        else if (hsv[0] >= GREEN_HUE_MIN && hsv[0] < GREEN_HUE_MAX)
                            ballColor = BallColor.Green;
                    }
                }
            }

            // Set hsv and time and return
            ballColor.setDistance(distance);
            ballColor.setHSV(hsv);
            ballColor.setTime(time);
            return ballColor;
        }

        public void telemetry(Telemetry telemetry, String label){
            if(telemetry==null || label == null)
                return;

            telemetry.addData(label + " color:", toString());
            telemetry.addData(label + " distance (cm):", distance);
            telemetry.addData(label + " hue:", hsv[0]);
            telemetry.addData(label + " saturation:", hsv[1]);
            telemetry.addData(label + " value:", hsv[2]);        }
    }
    private final DcMotor sortMotor;
    private final RevColorSensorV3 leftSensor;
    //private final RevColorSensorV3 rightSensor;
    //private final RevColorSensorV3 frontSensor;
    //private final RevColorSensorV3 backSensor;
    private int leftColor = 0; // 0 = none, 1 = green, 2 = purple
    private int lastLeftColor;
    private long lastLeftColorTime;
    private boolean isSpinning;

    private float redSensorValue;
    private float greenSensorValue;
    private float blueSensorValue;
    private long stopTime;
    private double targetPosition;
    private double greenDistance;
    private double purpleDistance;
    private double noneDistance;


    public static class Triplet {
        public float r, g, b;

        public Triplet(float r, float g, float b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
        public static double distance(Triplet color1, Triplet color2) {
            float distanceR = color1.r - color2.r;
            float distanceG = color1.g - color2.g;
            float distanceB = color1.b - color2.b;
            return Math.sqrt(Math.pow(distanceR, 2) + Math.pow(distanceG, 2) + Math.pow(distanceB, 2));
        }
    }

    Sorter.Triplet green = new Sorter.Triplet(0.05f,0.372f,0.0139f);
    Sorter.Triplet purple = new Sorter.Triplet(0.0296f,0.048f,0.0258f);
    Sorter.Triplet none = new Sorter.Triplet(0.0011f,0.0009f,0.0007f);

    public Sorter(StandardSetupOpMode ssom){
        this.ssom = ssom;
        sortMotor = ssom.hardwareMap.get(DcMotor.class, "sortMotor"); //need to define channel
        leftSensor = ssom.hardwareMap.get(RevColorSensorV3.class, "leftSensor"); // ic2 bus
        //rightSensor = ssom.hardwareMap.get(RevColorSensorV3.class, "rightSensor"); // ic2 bus
        //frontSensor = ssom.hardwareMap.get(RevColorSensorV3.class, "frontSensor"); // ic2 bus
        //backSensor = ssom.hardwareMap.get(RevColorSensorV3.class, "backSensor"); // ic2 bus

        leftSensor.setGain(10.0f);

        //isSpinning has to be false
        isSpinning = false;
        // Setup motor
        sortMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        sortMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sortMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sortMotor.setTargetPosition(0);
        sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Setup variables
        targetPosition = 0;

        // Debug
        boolean error = false;
        if(!leftSensor.isLightOn()) {
            ssom.telemetry.addLine("Error: Left color sensor light is not on!");
            error= true;
        }
        if(error)
        {
            ssom.telemetry.update();
        }
    }

    @Override
    public void run() {
        boolean pressed = false;
        setRunning();
        while (running) {
            // Sensor query
            NormalizedRGBA temp = leftSensor.getNormalizedColors();
            redSensorValue = temp.red;
            greenSensorValue = temp.green;
            blueSensorValue = temp.blue;

            Sorter.Triplet sensorColor = new Sorter.Triplet(redSensorValue, greenSensorValue, blueSensorValue);
            noneDistance = Triplet.distance(sensorColor, none);
            greenDistance = Triplet.distance(sensorColor, green);
            purpleDistance = Triplet.distance(sensorColor, purple);

            if (noneDistance < 0.01){
                leftColor = 0;
            }
            else if (greenSensorValue > blueSensorValue){
                leftColor = 1;
                lastLeftColor = 1;
                lastLeftColorTime = System.currentTimeMillis();
            }
            else {
                leftColor = 2;
                lastLeftColor = 2;
                lastLeftColorTime = System.currentTimeMillis();
            }

            // Listen for key presses
            if (!ssom.gamepadBuffer.ignoreGamepad) {
                if (!pressed && ssom.gamepadBuffer.g2DpadLeft) {
                    pressed = true;
                    targetPosition -= QUARTER_TURN;
                    isSpinning = true;
                }
                else if (!pressed && ssom.gamepadBuffer.g2DpadRight) {
                    pressed = true;
                    targetPosition += QUARTER_TURN;
                    isSpinning = true;
                }
                else if (!pressed && ssom.gamepadBuffer.g2DpadUp) {
                    pressed = true;
                    targetPosition += HALF_TURN;
                    isSpinning = true;
                }
                if (!ssom.gamepadBuffer.g2DpadLeft && !ssom.gamepadBuffer.g2DpadRight && !ssom.gamepadBuffer.g2DpadUp) {
                   pressed = false;
                   isSpinning = false;
                   stopTime = System.currentTimeMillis();
                }
            }

            // Are we close enough
            if(Math.abs(sortMotor.getCurrentPosition() - (int)Math.round(targetPosition)) < CLOSE_ENOUGH_TICKS)
                sortMotor.setPower(HOLD_POWER);
            else{
                sortMotor.setTargetPosition((int)Math.round(targetPosition));
                sortMotor.setPower(HALF_TURN_POWER);
            }
        }

        // Cleanup
        sortMotor.setPower(0);
    }

    //It checks if the last color time is a quarter of a second, and if not it assigns
    //black.
    protected int getColorForSide(long lastColorTime, int lastColor, long stopTime) {
        long diff = stopTime - lastColorTime;
        if (diff <= TAIL_ROTATE_TIME_MS) {
            return lastColor;
        } else {
            return 0;
        }
    }

    public boolean isSpinning(){
        return isSpinning;
    }

    /**
     * Rotate the left spot to the launcher
     */
    public void rotateLeftToLaunch(){
        targetPosition -= QUARTER_TURN;
    }

    /**
     * Rotate the right spot to the launcher
     */
    public void rotateRightToLaunch(){
        targetPosition += QUARTER_TURN;
    }

    /**
     * Rotate the front to the launcher
     */
    public void rotateFrontToLaunch(){
        targetPosition += HALF_TURN;
    }

    /**
     * Find the closest green and rotate it to the launcher
     * @return true if a green is in the hopper
     */
    public boolean rotateGreenToLaunch(){
        return true;
    }

    /**
     * Find the closest purple and rotate it to the launcher
     * @return true if a purple is in the hopper
     */
    public boolean rotatePurpleToLaunch(){
        return true;
    }

    @Override
    protected void setTo(SorterMetric metric) {

    }

    @Override
    public void safeHold() {

    }

    @Override
    protected boolean closeEnough(SorterMetric metric) {
        return false;
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        // I2C calls in telemetry can be very slow (only for debugging)
        telemetry.addData("leftRed", redSensorValue);
        telemetry.addData("leftGreen", greenSensorValue);
        telemetry.addData("leftBlue", blueSensorValue);
        //stelemetry.addData("leftDistance", leftSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("leftColor", (leftColor == 0) ? "None" : (leftColor == 1) ? "green" : "purple");
        telemetry.addData("lastLeftColor", lastLeftColor);
        telemetry.addData("lastLeftColorTime", lastLeftColorTime);
        telemetry.addData("sortMotor Ticks", sortMotor.getCurrentPosition());
        telemetry.addData("sortMotor Power",sortMotor.getPower());
        telemetry.addData("isSpinning", isSpinning);
        telemetry.addData("stopTime", stopTime);
        telemetry.addData("greenValue", greenSensorValue);
        telemetry.addData("blueValue", blueSensorValue);
        telemetry.addData("redValue", redSensorValue);
        telemetry.addData("greenDistance", greenDistance);
        telemetry.addData("purpleDistance", purpleDistance);
        telemetry.addData("noneDistance", noneDistance);
    }
}

