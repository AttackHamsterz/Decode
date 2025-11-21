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
    private static final long LAST_COLOR_WAIT_MS = 250;

    private static float redSensorValue;
    private static float greenSensorValue;
    private static float blueSensorValue;

    private static float hueValue;
    private static float saturationValue;
    private static float brightnessValue;

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
        private static final float PURPLE_HUE_MIN = 190;
        private static final float VALUE_MIN = 0.09f;

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

                    //Setting the telemetry.
                    redSensorValue = colors.red * 255;
                    greenSensorValue = colors.green * 255;
                    blueSensorValue = colors.blue * 255;
                    hueValue = hsv[0];
                    saturationValue = hsv[1];
                    brightnessValue = hsv[2];

                    //Checking the brightness to see if it's high enough to be a color value.
                    if (hsv[2] >= VALUE_MIN) {
                        if (hsv[0] >= PURPLE_HUE_MIN)
                            ballColor = BallColor.Purple;
                        else
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
            telemetry.addData(label + " value:", hsv[2]);

        }
    }
    private final DcMotor sortMotor;
    private final RevColorSensorV3 leftSensor;
    private final RevColorSensorV3 rightSensor;
    private final RevColorSensorV3 frontSensor;
    private final RevColorSensorV3 backSensor;
    private BallColor leftColor = BallColor.None;
    private BallColor lastLeftColor;
    private long lastLeftColorTime;

    private BallColor rightColor = BallColor.None;
    private BallColor lastRightColor;
    private long lastRightColorTime;

    private BallColor frontColor = BallColor.None;
    private BallColor lastFrontColor;
    private long lastFrontColorTime;

    private BallColor backColor = BallColor.None;
    private BallColor lastBackColor;
    private long lastBackColorTime;

    private boolean isSpinning;
    private long stopTime;
    private double targetPosition;


    public Sorter(StandardSetupOpMode ssom){
        this.ssom = ssom;
        sortMotor = ssom.hardwareMap.get(DcMotor.class, "sortMotor"); //need to define channel
        leftSensor = ssom.hardwareMap.get(RevColorSensorV3.class, "leftSensor"); // ic2 bus
        rightSensor = ssom.hardwareMap.get(RevColorSensorV3.class, "rightSensor"); // ic2 bus
        frontSensor = ssom.hardwareMap.get(RevColorSensorV3.class, "frontSensor"); // ic2 bus
        backSensor = ssom.hardwareMap.get(RevColorSensorV3.class, "backSensor"); // ic2 bus

        leftSensor.setGain(20.0f);
        rightSensor.setGain(20.0f);
        frontSensor.setGain(20.0f);
        backSensor.setGain(20.0f);

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
        if(!rightSensor.isLightOn()) {
            ssom.telemetry.addLine("Error: Right color sensor light is not on!");
            error= true;
        }
        if(!frontSensor.isLightOn()) {
            ssom.telemetry.addLine("Error: Front color sensor light is not on!");
            error= true;
        }
        if(!backSensor.isLightOn()) {
            ssom.telemetry.addLine("Error: Back color sensor light is not on!");
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
            // Getting the color value from the HSV code in BallColor.
            leftColor = BallColor.fromSensor(leftSensor);
            rightColor = BallColor.fromSensor(rightSensor);
            frontColor = BallColor.fromSensor(frontSensor);
            backColor = BallColor.fromSensor(backSensor);

            if (leftColor == BallColor.Green) {
                 lastLeftColor = BallColor.Green;
                lastLeftColorTime = System.currentTimeMillis();
            } else if (leftColor == BallColor.Purple) {
                lastLeftColor = BallColor.Purple;
                lastLeftColorTime = System.currentTimeMillis();
            }
            if (rightColor == BallColor.Green) {
                lastRightColor = BallColor.Green;
                lastRightColorTime = System.currentTimeMillis();
            } else if (rightColor == BallColor.Purple) {
                lastRightColor = BallColor.Purple;
                lastRightColorTime = System.currentTimeMillis();
            }
            if (frontColor == BallColor.Green) {
                lastFrontColor = BallColor.Green;
                lastFrontColorTime = System.currentTimeMillis();
            } else if (frontColor == BallColor.Purple) {
                lastFrontColor = BallColor.Purple;
                lastFrontColorTime = System.currentTimeMillis();
            }
            if (backColor == BallColor.Green) {
                lastBackColor = BallColor.Green;
                lastBackColorTime = System.currentTimeMillis();
            } else if (backColor == BallColor.Purple) {
                lastBackColor = BallColor.Purple;
                lastBackColorTime = System.currentTimeMillis();
            }

            // Listen for key presses
            if (!ssom.gamepadBuffer.ignoreGamepad) {
                if (!pressed && ssom.gamepadBuffer.g2DpadLeft) {
                    pressed = true;
                    targetPosition -= QUARTER_TURN;
                }
                else if (!pressed && ssom.gamepadBuffer.g2DpadRight) {
                    pressed = true;
                    targetPosition += QUARTER_TURN;
                }
                else if (!pressed && ssom.gamepadBuffer.g2DpadUp) {
                    pressed = true;
                    targetPosition += HALF_TURN;
                }
                //Logic to move a selected color to the launcher
                else if(!pressed && ssom.gamepadBuffer.g2LeftBumper){
                    pressed = true;
                    rotateGreenToLaunch();
                }
                else if(!pressed && ssom.gamepadBuffer.g2RightBumper){
                    pressed = true;
                    rotatePurpleToLaunch();
                }
                if (!ssom.gamepadBuffer.g2DpadLeft && !ssom.gamepadBuffer.g2DpadRight && !ssom.gamepadBuffer.g2DpadUp && !ssom.gamepadBuffer.g2LeftBumper && !ssom.gamepadBuffer.g2RightBumper) {
                   pressed = false;
                }
            }

            // Are we close enough
            if(Math.abs(sortMotor.getCurrentPosition() - (int)Math.round(targetPosition)) < CLOSE_ENOUGH_TICKS) {
                sortMotor.setPower(HOLD_POWER);
                isSpinning = false;
                stopTime = System.currentTimeMillis();
            }
            else{
                isSpinning = true;
                sortMotor.setTargetPosition((int)Math.round(targetPosition));
                sortMotor.setPower(HALF_TURN_POWER);
            }
        }

        // Cleanup
        sortMotor.setPower(0);
    }

    public boolean isSpinning(){
        return isSpinning;
    }

    /**
     * Find the closest green and rotate it to the launcher
     * @return true if a green is in the hopper
     */
    public boolean rotateGreenToLaunch(){
        boolean colorFound = false;
        long recent = stopTime - LAST_COLOR_WAIT_MS;

        if (backColor == BallColor.Green ||
            (lastBackColor == BallColor.Green && lastBackColorTime > recent)) {
            // Don't do anything.
            colorFound = true;
        }
        else if(leftColor == BallColor.Green ||
                (lastLeftColor == BallColor.Green && lastLeftColorTime > recent)){
            colorFound = true;
            rotateClockwise(-1);
        }
        else if(rightColor == BallColor.Green ||
                (lastRightColor == BallColor.Green && lastRightColorTime > recent)){
            colorFound = true;
                rotateClockwise(1);
        }
        else if(frontColor == BallColor.Green ||
                (lastFrontColor == BallColor.Green && lastFrontColorTime > recent)){
            colorFound = true;
            rotateClockwise(-2);
        }
        return colorFound;
    }

    /**
     * Find the closest purple and rotate it to the launcher
     * @return true if a purple is in the hopper
     */
    public boolean rotatePurpleToLaunch(){
        boolean colorFound = false;
        long recent = stopTime - LAST_COLOR_WAIT_MS;

        if (backColor == BallColor.Purple ||
                (lastBackColor == BallColor.Purple && lastBackColorTime > recent)) {
           // Don't do anything.
            colorFound = true;
        }
        else if(leftColor == BallColor.Purple||
                (lastLeftColor == BallColor.Purple && lastLeftColorTime > recent)) {
            rotateClockwise(-1);
            colorFound = true;
        }
        else if(rightColor == BallColor.Purple ||
                (lastRightColor == BallColor.Purple && lastRightColorTime > recent)){
            rotateClockwise(1);
            colorFound = true;
        }
        else if(frontColor == BallColor.Purple ||
                (lastFrontColor == BallColor.Purple && lastFrontColorTime > recent)){
            rotateClockwise(-2);
            colorFound = true;
        }
        return colorFound;
    }

    /**
     * Rotate the current position by the specified number of quarter turns
     * @param quarterTurns number of clockwise quarter turns
     */
    public void rotateClockwise(int quarterTurns) {
        isSpinning = true;
        targetPosition += quarterTurns*QUARTER_TURN;
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
        if((DEBUG & 8) != 0) {
            // I2C calls in telemetry can be very slow (only for debugging)
            //telemetry.addData("leftDistance", leftSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("leftColor", (leftColor == BallColor.None) ? "None" : (leftColor == BallColor.Green) ? "green" : "purple");
            telemetry.addData("rightColor", (rightColor == BallColor.None) ? "None" : (rightColor == BallColor.Green) ? "green" : "purple");
            telemetry.addData("frontColor", (frontColor == BallColor.None) ? "None" : (frontColor == BallColor.Green) ? "green" : "purple");
            telemetry.addData("backColor", (backColor == BallColor.None) ? "None" : (backColor == BallColor.Green) ? "green" : "purple");
            telemetry.addData("sortMotor Ticks", sortMotor.getCurrentPosition());
            telemetry.addData("sortMotor Power", sortMotor.getPower());
            telemetry.addData("isSpinning", isSpinning);
            /*
            telemetry.addData("lastLeftColor", lastLeftColor);
            telemetry.addData("lastLeftColorTime", lastLeftColorTime);
            telemetry.addData("lastRightColor", lastRightColor);
            telemetry.addData("lastRightColorTime", lastRightColorTime);
            telemetry.addData("lastFrontColor", lastFrontColor);
            telemetry.addData("lastFrontColorTime", lastFrontColorTime);
            telemetry.addData("lastBackColor", lastBackColor);
            telemetry.addData("lastBackColorTime", lastBackColorTime);
            telemetry.addData("stopTime", stopTime);
             */
        }
    }
}

