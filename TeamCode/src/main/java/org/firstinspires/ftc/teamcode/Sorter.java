package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Sorter extends RobotPart<SorterMetric>{
    private static final double PPM = 1425.1;
    private static final double HALF_TURN = PPM / 2.0;
    private static final double QUARTER_TURN = PPM / 4.0;
    private static final int CLOSE_ENOUGH_TICKS = 14;
    private static final double HOLD_POWER = 0.1;
    private static final double TAIL_ROTATE_TIME_MS = 100;
    private static final double QUARTER_TURN_POWER = 1.0;
    private static final double HALF_TURN_POWER = 1.0;

    private final DcMotorEx sortMotor;
    private final RevColorSensorV3 leftSensor;
    //private final RevColorSensorV3 rightSensor;
    //private final RevColorSensorV3 frontSensor;
    //private final RevColorSensorV3 backSensor;
    private int leftColor = 0; // 0 = none, 1 = green, 2 = purple
    private int lastLeftColor;
    private long lastLeftColorTime;
    private boolean isSpinning;
    private long stopTime;
    private double targetPosition;

    public static class Triplet {
        public int r, g, b;

        public Triplet(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
        public static double distance(Triplet color1, Triplet color2) {
            int distanceR = color1.r - color2.r;
            int distanceG = color1.g - color2.g;
            int distanceB = color1.b - color2.b;
            return Math.sqrt(distanceR^2 + distanceG^2 + distanceB^2);
        }
    };

    Sorter.Triplet green = new Sorter.Triplet(862,3226,2385);
    Sorter.Triplet purple = new Sorter.Triplet(1594,1767,3050);
    Sorter.Triplet none = new Sorter.Triplet(0,0,0);

    public Sorter(StandardSetupOpMode ssom, boolean ignoreGamepad){
        this.ssom = ssom;
        this.ignoreGamepad = ignoreGamepad;
        sortMotor = ssom.hardwareMap.get(DcMotorEx.class, "sortMotor"); //need to define channel
        leftSensor = ssom.hardwareMap.get(RevColorSensorV3.class, "leftSensor"); // ic2 bus
        //rightSensor = ssom.hardwareMap.get(RevColorSensorV3.class, "rightSensor"); // ic2 bus
        //frontSensor = ssom.hardwareMap.get(RevColorSensorV3.class, "frontSensor"); // ic2 bus
        //backSensor = ssom.hardwareMap.get(RevColorSensorV3.class, "backSensor"); // ic2 bus

        // Setup motor
        sortMotor.setDirection(DcMotorEx.Direction.FORWARD);
        //sortMotor.setPositionPIDFCoefficients(5.0);
        sortMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sortMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sortMotor.setTargetPosition(0);
        sortMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

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
        while (!isInterrupted()) {
            // Sensor query
            int redValue = leftSensor.red();
            int greenValue = leftSensor.green();
            int blueValue = leftSensor.blue();

            Sorter.Triplet sensorColor = new Sorter.Triplet(redValue, greenValue, blueValue);

            if (Triplet.distance(sensorColor, none) <500){
                //letfColor = 0;
            }
            else if (Triplet.distance(sensorColor, green) < Triplet.distance(sensorColor, purple)){
                lastLeftColor = 1;
                lastLeftColorTime = System.currentTimeMillis();
            }
            else if (Triplet.distance(sensorColor, green) > Triplet.distance(sensorColor, purple)){
                lastLeftColor = 2;
                lastLeftColorTime = System.currentTimeMillis();
            }

            // Listen for key presses
            if (!ignoreGamepad) {
                if (!pressed && ssom.gamepadBuffer.g2DpadLeft) {
                    pressed = true;
                    targetPosition -= QUARTER_TURN;
                    sortMotor.setTargetPosition((int)Math.round(targetPosition));
                    sortMotor.setPower(QUARTER_TURN_POWER);
                }
                else if (!pressed && ssom.gamepadBuffer.g2DpadRight) {
                    pressed = true;
                    targetPosition += QUARTER_TURN;
                    sortMotor.setTargetPosition((int)Math.round(targetPosition));
                    sortMotor.setPower(QUARTER_TURN_POWER);
                }
                else if (!pressed && ssom.gamepadBuffer.g2DpadUp) {
                    pressed = true;
                    targetPosition += HALF_TURN;
                    sortMotor.setTargetPosition((int)Math.round(targetPosition));
                    sortMotor.setPower(HALF_TURN_POWER);
                }
                if (!ssom.gamepadBuffer.g2DpadLeft && !ssom.gamepadBuffer.g2DpadRight && !ssom.gamepadBuffer.g2DpadUp) {
                   pressed = false;
                }
            }

            // Are we close enough
            if(Math.abs(sortMotor.getCurrentPosition() - (int)Math.round(targetPosition)) < CLOSE_ENOUGH_TICKS)
                sortMotor.setPower(HOLD_POWER);

            checkIfSidesHaveColors();
        }
    }

    // It's 'supposed' to check to see if the motor is spinning and if stopped get
    // the color of the balls... heh...
    //To do getPower() is currently being stupid and not changing into decimals
    //...what the heck... anyway
    protected void checkIfSidesHaveColors(){
        if (sortMotor.getPower() <= HOLD_POWER) {
            if (isSpinning) {
                stopTime = System.currentTimeMillis();
                leftColor = getColorForSide(lastLeftColorTime, lastLeftColor, stopTime);
                //rightColor = getColorForSide(lastRightColorTime, lastRightColor, stopTime);
                //frontColor = getColorForSide(lastFrontColorTime, lastFrontColor, stopTime);
                //backColor = getColorForSide(lastBackColorTime, lastBackColor, stopTime);
            }
            isSpinning = false;
        } else {
            isSpinning = true;
        }
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
        //telemetry.addData("leftRed", leftSensor.red());
        //telemetry.addData("leftGreen", leftSensor.green());
        //telemetry.addData("leftBlue", leftSensor.blue());
        //telemetry.addData("leftDistance", leftSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("leftColor", (leftColor == 0) ? "None" : (leftColor == 1) ? "green" : "purple");
        telemetry.addData("lastLeftColor", lastLeftColor);
        telemetry.addData("lastLeftColorTime", lastLeftColorTime);
        telemetry.addData("sortMotor Ticks", sortMotor.getCurrentPosition());
        telemetry.addData("sortMotor Power",sortMotor.getPower());
        telemetry.addData("isSpinning", isSpinning);
        telemetry.addData("stopTime", stopTime);
    }
}

