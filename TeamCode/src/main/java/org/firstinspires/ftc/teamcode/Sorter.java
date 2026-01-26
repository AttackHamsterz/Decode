package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class Sorter extends RobotPart<SorterMetric>{
    private static final double PPM = 1425.1;
    private static final double QUARTER_TURN = PPM / 4.0;
    private static final double ONE_DEGREE_TURN = PPM / 360.0;
    private static final int CLOSE_ENOUGH_TICKS = (int)Math.ceil(3.0 * ONE_DEGREE_TURN);
    private static final double HOLD_POWER = 0.1;
    private static final double TURN_POWER = 1.0;
    private static final long STUCK_TIME_MS = 2000;
    private static final long FRONT_AUTO_TURN_DELAY_MS = 300;
    private static final long LEFT_AUTO_TURN_DELAY_MS = 300;
    private static final long RIGHT_AUTO_TURN_DELAY_MS = 300;
    private static final long TELE_TURN_DELAY_MS = 350;
    private static final double MIN_DIST_CM = 3.0;

    private long spinStartTime = 0;
    private boolean frontAutoTurn = false;
    private boolean leftAutoTurn = false;
    private boolean rightAutoTurn = false;
    private boolean autoTurnTrigger = false;

    // Position tracking array
    private final BallColor[] ballPositions = new BallColor[4];
    private int backIndex = 0;

    public enum BallColor{
        None(0),
        Green(1),
        Purple(2);

        private final String color;
        private double distance;
        private final float[] hsv;
        private long time;

        private static final float PURPLE_HUE_MIN = 190;
        private static final float VALUE_MIN = 0.09f;

        BallColor(int id) {
            if(id < 1 || id > 2) id = 0;
            this.color = (id == 1) ? "Green" : (id == 2) ? "Purple" : "None";
            this.hsv = new float[3];
            this.distance = 0;
            this.time = 0;
        }

        public void setHSV(float[] newHsv){
            if(newHsv.length > 2) {
                this.hsv[0] = newHsv[0];
                this.hsv[1] = newHsv[1];
                this.hsv[2] = newHsv[2];
            }
        }

        public void setDistance(double distance){
            this.distance = distance;
        }

        public void setTime(long time){
            this.time = time;
        }

        /* These aren't used
        public long getTime(){
            return time;
        }


        public double getDistance(){
            return distance;
        }
        */

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

                    //Checking the brightness to see if it's high enough to be a color value.
                    if (hsv[2] >= VALUE_MIN) {
                        if (hsv[0] >= PURPLE_HUE_MIN)
                            ballColor = BallColor.Purple;
                        else
                            ballColor = BallColor.Green;
                    }
                    else
                        ballColor = BallColor.Purple;
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

    private boolean isSpinning;
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

        // Initialize ball position tracking array to None
        ballPositions[0] = BallColor.None; // front
        ballPositions[1] = BallColor.None; // left
        ballPositions[2] = BallColor.None; // right
        ballPositions[3] = BallColor.None; // back

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

    public void frontAutoTurnOn(){
        frontAutoTurn = true;
    }
    public void frontAutoTurnOff(){
        frontAutoTurn = false;
    }

    public void frontAutoTurnThread(){
        autoTurnTrigger = true;
        ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);
        Runnable task = () -> {
            // Only do the auto rotation if we're not intaking on the sides
            if(ssom.intake.isLeftOff() && ssom.intake.isRightOff())
                rotateClockwise(-1);
            autoTurnTrigger = false;
        };
        long turnDelay = ssom.gamepadBuffer.ignoreGamepad? FRONT_AUTO_TURN_DELAY_MS: TELE_TURN_DELAY_MS;
        scheduler.schedule(task, turnDelay, TimeUnit.MILLISECONDS);
    }

    public void leftAutoTurnOn(){
        leftAutoTurn = true;
    }
    public void leftAutoTurnOff(){
        leftAutoTurn = false;
    }

    public void leftAutoTurnThread(){
        autoTurnTrigger = true;
        ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);
        Runnable task = () -> {
            // Only do the auto rotation if we're not intaking on the sides
            if(ssom.intake.isFrontOff() && ssom.intake.isRightOff())
                rotateClockwise(-1);
            autoTurnTrigger = false;
        };
        long turnDelay = ssom.gamepadBuffer.ignoreGamepad? LEFT_AUTO_TURN_DELAY_MS: TELE_TURN_DELAY_MS;
        scheduler.schedule(task, turnDelay, TimeUnit.MILLISECONDS);
    }

    public void rightAutoTurnOn(){
        rightAutoTurn = true;
    }
    public void rightAutoTurnOff(){
        rightAutoTurn = false;
    }

    public void rightAutoTurnThread(){
        autoTurnTrigger = true;
        ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);
        Runnable task = () -> {
            // Only do the auto rotation if we're not intaking on the sides
            if(ssom.intake.isLeftOff() && ssom.intake.isFrontOff())
                rotateClockwise(-1);
            autoTurnTrigger = false;
        };
        long turnDelay = ssom.gamepadBuffer.ignoreGamepad? RIGHT_AUTO_TURN_DELAY_MS: TELE_TURN_DELAY_MS;
        scheduler.schedule(task, turnDelay, TimeUnit.MILLISECONDS);
    }

    // Method for BallLifter to call when a ball is launched (only method that clears color values)
    public void clearBackPosition() {
        ballPositions[backIndex] = BallColor.None;
    }

    private int safeIndex(int delta){
        int index = (backIndex + delta) % 4;
        if(index < 0)
            index += 4;
        return index;
    }

    // Given a motor position and sensor location, returns which spot (0-3) that sensor was observing
    private int getObservedPos(int motorPosition, int sensorOffset) {

        double fractionalPosition = (double) motorPosition / QUARTER_TURN;
        int backSpot = (int) Math.round(fractionalPosition) % 4;
        if (backSpot < 0) backSpot += 4;

        // Then offset to get the Spot this sensor is looking at
        int spot = (backSpot + sensorOffset) % 4;
        if (spot < 0) spot += 4;
        return spot;
    }

    @Override
    public void run() {
        boolean pressed = false;
        setRunning();

        while (running) {
            int leftMotorPos = sortMotor.getCurrentPosition();
            BallColor leftColor = BallColor.fromSensor(leftSensor);
            int leftObservedSpot = getObservedPos(leftMotorPos, -1);

            int rightMotorPos = sortMotor.getCurrentPosition();
            BallColor rightColor = BallColor.fromSensor(rightSensor);
            int rightObservedSpot = getObservedPos(rightMotorPos, +1);

            int frontMotorPos = sortMotor.getCurrentPosition();
            BallColor frontColor = BallColor.fromSensor(frontSensor);
            int frontObservedSpot = getObservedPos(frontMotorPos, +2);

            int backMotorPos = sortMotor.getCurrentPosition();
            BallColor backColor = BallColor.fromSensor(backSensor);
            int backObservedSpot = getObservedPos(backMotorPos, 0);

            // Update backIndex for use by other methods
            backIndex = backObservedSpot;

            // Store colors in the Spots each sensor actually observed
            if (leftColor != BallColor.None) {
                ballPositions[leftObservedSpot] = leftColor;
                if (leftColor.distance <= MIN_DIST_CM && !isSpinning && leftAutoTurn && !autoTurnTrigger)
                    leftAutoTurnThread();
            }
            if (rightColor != BallColor.None) {
                ballPositions[rightObservedSpot] = rightColor;
                if (rightColor.distance <= MIN_DIST_CM && !isSpinning && rightAutoTurn && !autoTurnTrigger)
                    rightAutoTurnThread();
            }
            if (frontColor != BallColor.None) {
                ballPositions[frontObservedSpot] = frontColor;
                if (frontColor.distance <= MIN_DIST_CM && !isSpinning && frontAutoTurn && !autoTurnTrigger)
                    frontAutoTurnThread();
            }
            if (backColor != BallColor.None) {
                ballPositions[backObservedSpot] = backColor;
            }

            // Listen for key presses
            if (!ssom.gamepadBuffer.ignoreGamepad) {
                // Since we are usually behind the robot, the following makes sense:
                //   x rotates left quad to launcher
                //   y rotates front quad to launcher
                //   b rotates right quad to launcher
                //   a rotates closest anything to launcher
                if (!pressed && ssom.gamepadBuffer.g2b) {
                    pressed = true;
                    rotateClockwise(1);
                }
                else if (!pressed && ssom.gamepadBuffer.g2x) {
                    pressed = true;
                    rotateClockwise(-1);
                }
                else if (!pressed && ssom.gamepadBuffer.g2y) {
                    pressed = true;
                    rotateClockwise(2);
                }
                else if (!pressed && ssom.gamepadBuffer.g2a) {
                    pressed = true;
                    if(!backOccupied()) {
                        if (rightOccupied())
                            rotateClockwise(1);
                        else if (leftOccupied())
                            rotateClockwise(-1);
                        else if (frontOccupied())
                            rotateClockwise(2);
                    }
                }
                // Logic to nudge sorter for alignment
                else if (!pressed && ssom.gamepadBuffer.g2DpadLeft) {
                    pressed = true;
                    targetPosition += 2.0 * ONE_DEGREE_TURN;
                }
                else if (!pressed && ssom.gamepadBuffer.g2DpadRight) {
                    pressed = true;
                    targetPosition -= 2.0 * ONE_DEGREE_TURN;
                }
                // Logic to move a selected color to the launcher (left->green, right->purple)
                else if(!pressed && ssom.gamepadBuffer.g2LeftBumper){
                    pressed = true;
                    rotateGreenToLaunch();
                }
                else if(!pressed && ssom.gamepadBuffer.g2RightBumper){
                    pressed = true;
                    rotatePurpleToLaunch();
                }
                if (!ssom.gamepadBuffer.g2x && !ssom.gamepadBuffer.g2y && !ssom.gamepadBuffer.g2a && !ssom.gamepadBuffer.g2b &&
                        !ssom.gamepadBuffer.g2DpadLeft && !ssom.gamepadBuffer.g2DpadRight &&
                        !ssom.gamepadBuffer.g2LeftBumper && !ssom.gamepadBuffer.g2RightBumper) {
                    pressed = false;
                }
            }

            // Are we close enough
            if(Math.abs(backMotorPos - (int)Math.round(targetPosition)) < CLOSE_ENOUGH_TICKS) {
                sortMotor.setPower(HOLD_POWER);
                isSpinning = false;
                spinStartTime = 0;
            }
            // Are we stuck - adjust pendingQuarterTurns when reversing direction
            else if(spinStartTime > 0 && System.currentTimeMillis() - spinStartTime > STUCK_TIME_MS){
                if(targetPosition > sortMotor.getCurrentPosition()) {
                    rotateClockwise(-1);
                } else {
                    rotateClockwise(1);
                }
                spinStartTime = 0;
            }
            // Spin to target position if not lifting
            else if (!ssom.ballLifter.isLifting()){
                if(spinStartTime == 0)
                    spinStartTime = System.currentTimeMillis();
                isSpinning = true;
                sortMotor.setTargetPosition((int)Math.round(targetPosition));
                sortMotor.setPower(TURN_POWER);
            }
        }

        // Cleanup
        sortMotor.setPower(0);
    }

    public boolean isNotSpinning(){
        return !isSpinning;
    }

    /**
     * Find the closest green and rotate it to the launcher
     * @return true if a green is in the hopper
     */
    public boolean rotateGreenToLaunch(){
        boolean colorFound = false;

        // Check position tracking array and sensor readings
        int leftIndex = safeIndex(-1);
        int rightIndex = safeIndex(1);
        int frontIndex = safeIndex(2);
        if (ballPositions[backIndex] == BallColor.Green) {
            // Don't do anything.
            colorFound = true;
        }
        else if(ballPositions[leftIndex] == BallColor.Green){
            colorFound = true;
            rotateClockwise(-1);
        }
        else if(ballPositions[rightIndex] == BallColor.Green){
            colorFound = true;
            rotateClockwise(1);
        }
        else if(ballPositions[frontIndex] == BallColor.Green){
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

        // Check position tracking array and sensor readings
        int leftIndex = safeIndex(-1);
        int rightIndex = safeIndex(1);
        int frontIndex = safeIndex(2);
        if (ballPositions[backIndex] == BallColor.Purple) {
            // Don't do anything.
            colorFound = true;
        }
        else if(ballPositions[leftIndex] == BallColor.Purple) {
            rotateClockwise(-1);
            colorFound = true;
        }
        else if(ballPositions[rightIndex] == BallColor.Purple){
            rotateClockwise(1);
            colorFound = true;
        }
        else if(ballPositions[frontIndex] == BallColor.Purple){
            rotateClockwise(-2);
            colorFound = true;
        }
        return colorFound;
    }

    // Occupied methods check both sensor AND position tracking
    public boolean frontOccupied(){
        int frontIndex = safeIndex(2);
        return ballPositions[frontIndex] != BallColor.None;
    }

    public boolean leftOccupied(){
        int leftIndex = safeIndex(-1);
        return ballPositions[leftIndex] != BallColor.None;
    }

    public boolean rightOccupied(){
        int rightIndex = safeIndex(1);
        return ballPositions[rightIndex] != BallColor.None;
    }

    public boolean backOccupied(){
        return ballPositions[backIndex] != BallColor.None;
    }

    public int getBallCount(){
        int count = 0;
        if(frontOccupied())
            count++;
        if(leftOccupied())
            count++;
        if(rightOccupied())
            count++;
        if(backOccupied())
            count++;
        return count;
    }

    public void emptyFront(){
        if(frontOccupied()){
            if(!rightOccupied())
                rotateClockwise(-1);
            else if(!leftOccupied())
                rotateClockwise(1);
            else if(!backOccupied())
                rotateClockwise(-2);
        }
    }

    public void emptyLeft(){
        if(leftOccupied()){
            if(!frontOccupied())
                rotateClockwise(-1);
            else if(!backOccupied())
                rotateClockwise(1);
            else if(!rightOccupied())
                rotateClockwise(-2);
        }
    }

    public void emptyRight(){
        if(rightOccupied()){
            if(!backOccupied())
                rotateClockwise(-1);
            else if(!frontOccupied())
                rotateClockwise(1);
            else if(!leftOccupied())
                rotateClockwise(-2);
        }
    }

    /**
     * Rotate the current position by the specified number of quarter turns
     * @param quarterTurns number of quarter turns (positive = left→front, negative = front→left)
     */
    public void rotateClockwise(int quarterTurns) {
        isSpinning = true;
        targetPosition += quarterTurns * QUARTER_TURN;
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
            // Position tracking telemetry
            int backIndex = this.backIndex;
            int leftIndex = safeIndex(-1);
            int rightIndex = safeIndex(1);
            int frontIndex = safeIndex(2);
            telemetry.addLine("=== Position Array ===");
            telemetry.addData("ballPositions (Front)", ballPositions[frontIndex]);
            telemetry.addData("ballPositions (Left)", ballPositions[leftIndex]);
            telemetry.addData("ballPositions (Back)", ballPositions[backIndex]);
            telemetry.addData("ballPositions (Right)", ballPositions[rightIndex]);
            telemetry.addData("backIndex", backIndex);
        }
    }
}