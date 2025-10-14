package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sorter extends RobotPart<SorterMetric>{
    private static final double PPM = 1425.1;
    private final DcMotor sortMotor;
    private final RevColorSensorV3 leftSensor;
    //private final RevColorSensorV3 rightSensor;
    //private final RevColorSensorV3 frontSensor;
    private int leftColor = 0; // 0 = none, 1 = green, 2 = purple
    private int lastLeftColor;
    private long lastLeftColorTime;
    private boolean isSpinning;
    private long stopTime;
    private double currentPosition;

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
        this.gamepad = ssom.gamepad2;
        this.ignoreGamepad = ignoreGamepad;
        sortMotor = ssom.hardwareMap.get(DcMotor.class, "sortMotor"); //need to define channel
        leftSensor = ssom.hardwareMap.get(RevColorSensorV3.class, "leftSensor"); // ic2 bus
        //rightSensor = ssom.hardwareMap.get(RevColorSensorV3.class, "rightSensor"); // ic2 bus
        //frontSensor = ssom.hardwareMap.get(RevColorSensorV3.class, "frontSensor"); // ic2 bus

        // Setup motor
        sortMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        sortMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sortMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sortMotor.setTargetPosition(0);
        sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sortMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Setup variables
        currentPosition = 0;
    }

    @Override
    public void run() {
        boolean pressed = false;
        while (!isInterrupted()) {
            // Sensor query
            leftSensor.getDistance(DistanceUnit.CM);
            leftSensor.green();
            leftSensor.red();
            leftSensor.blue();

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
                if (!pressed && gamepad.a) {
                    pressed = true;
                    currentPosition += 1425.1/4.0;
                    sortMotor.setTargetPosition((int)Math.round(currentPosition));
                    sortMotor.setPower(1);
                }
                else if (!pressed && gamepad.b) {
                    pressed = true;
                    currentPosition -= 1425.1/4.0;
                    sortMotor.setTargetPosition((int)Math.round(currentPosition));
                    sortMotor.setPower(1);
                }
                if (!gamepad.a && !gamepad.b) {
                   pressed = false;
                }
            }

            checkIfSidesHaveColors();
        }
    }

    // It's 'supposed' to check to see if the motor is spinning and if stopped get
    // the color of the balls... heh...
    //To do getPower() is currently being stupid and not changing into decimals
    //...what the heck... anyway
    protected void checkIfSidesHaveColors(){
        if (Math.abs(sortMotor.getPower()) <= 0.01) {
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
        if (diff <= 0.25) {
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
        telemetry.addData("leftRed", leftSensor.red());
        telemetry.addData("leftGreen", leftSensor.green());
        telemetry.addData("leftBlue", leftSensor.blue());
        telemetry.addData("sorterticks", sortMotor.getCurrentPosition());
        telemetry.addData("leftDistance", leftSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("leftColor", (leftColor == 0) ? "None" : (leftColor == 1) ? "green" : "purple");
        telemetry.addData("lastLeftColor", lastLeftColor);
        telemetry.addData("lastLeftColorTime", lastLeftColorTime);
        telemetry.addData("isSpinning", isSpinning);
        telemetry.addData("stopTime", stopTime);
        telemetry.addData("sortMotor Power",sortMotor.getPower());
    }
}

