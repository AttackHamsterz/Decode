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
    private final RevColorSensorV3 rightSensor;
    private final RevColorSensorV3 frontSensor;

    public Sorter(StandardSetupOpMode ssom, boolean ignoreGamepad){
        this.ssom = ssom;
        this.gamepad = ssom.gamepad2;
        this.ignoreGamepad = ignoreGamepad;
        sortMotor = ssom.hardwareMap.get(DcMotor.class, "sortMotor"); //need to define channel
        leftSensor = ssom.hardwareMap.get(RevColorSensorV3.class, "leftSensor"); // ic2 bus
        rightSensor = ssom.hardwareMap.get(RevColorSensorV3.class, "rightSensor"); // ic2 bus
        frontSensor = ssom.hardwareMap.get(RevColorSensorV3.class, "frontSensor"); // ic2 bus

        // Setup motor
        sortMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        sortMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sortMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sortMotor.setTargetPosition(0);
        sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            // Sensor query
            leftSensor.getDistance(DistanceUnit.CM);
            leftSensor.green();
            leftSensor.red();
            leftSensor.blue();

            // Listen for key presses
            if (!ignoreGamepad) {
                if (gamepad.a) {
                    sortMotor.setPower(1);
                    sortMotor.setTargetPosition(1425); //1425.1
                }
                else if (gamepad.b) {
                    sortMotor.setPower(1);
                    sortMotor.setTargetPosition(0);
                }
                else if (gamepad.start) {
                    sortMotor.setPower(0);
                }
            }
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
        telemetry.addData("sorterticks", sortMotor.getCurrentPosition());
        telemetry.addData("leftDistance", leftSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("leftRed", leftSensor.red());
        telemetry.addData("leftGreen", leftSensor.green());
        telemetry.addData("leftBlue", leftSensor.blue());
    }
}

