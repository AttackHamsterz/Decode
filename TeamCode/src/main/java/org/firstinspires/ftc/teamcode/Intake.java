package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends RobotPart<IntakeMetric> {
    private static final double ENOUGH_JOYSTICK = 0.1;
    private static final double UPPER_SERVO_POWER = 1;
    private static final double LOWER_SERVO_POWER = 1;
    private final CRServo leftIntakeServo;
    private final CRServo frontIntakeServo;
    private final CRServo rightIntakeServo;
    //private final CRServo leftUpperServo;
    //private final CRServo frontUpperServo;
    private final CRServo rightUpperServo;
    private double leftIntakeServoPower;
    private double rightIntakeServoPower;
    private double frontIntakeServoPower;
    // private double leftUpperServoPower;
    // private double rightUpperServoPower;
    private double rightUpperServoPower;


    public Intake(StandardSetupOpMode ssom) {

        this.ssom = ssom;
        leftIntakeServo = ssom.hardwareMap.get(CRServo.class, "leftIntakeServo"); //need to define channel
        rightIntakeServo = ssom.hardwareMap.get(CRServo.class, "rightIntakeServo"); //need to define channel
        frontIntakeServo = ssom.hardwareMap.get(CRServo.class, "frontIntakeServo");//need to define channel
        //leftUpperServo = ssom.hardwareMap.get(CRServo.class, "leftUpperServo");
        //rightUpperServo = ssom.hardwareMap.get(CRServo.class, "rightUpperServo");
        rightUpperServo = ssom.hardwareMap.get(CRServo.class, "rightUpperServo");

        leftIntakeServo.setDirection(CRServo.Direction.FORWARD);
        rightIntakeServo.setDirection(CRServo.Direction.FORWARD);
        frontIntakeServo.setDirection(CRServo.Direction.FORWARD);
        //leftUpperServo.setDirection(CRServo.Direction.REVERSE);
        //rightUpperServo.setDirection(CRServo.Direction.REVERSE);
        rightUpperServo.setDirection(CRServo.Direction.REVERSE);

        leftIntakeServoPower = 0;
        rightIntakeServoPower = 0;
        frontIntakeServoPower = 0;
        //leftIntakeServoPower = 0;
        //rightIntakeServoPower = 0;
        rightUpperServoPower = 0;

    }

    @Override
    public void run() {
        boolean emptyLeft = false;
        boolean emptyRight = false;
        boolean emptyFront = false;
        setRunning();
        while (running) {
            if (!ssom.gamepadBuffer.ignoreGamepad) {
                // Left intake (outtake wins)
                if (ssom.gamepadBuffer.g2RightStickX < -ENOUGH_JOYSTICK || ssom.gamepadBuffer.g2RightStickY > ENOUGH_JOYSTICK) {
                    leftIntakeServoPower = -1;
                    //leftUpperServoPower = -UPPER_SERVO_POWER;
                } else if (ssom.gamepadBuffer.g2LeftStickX < -ENOUGH_JOYSTICK) {
                    leftIntakeServoPower = 1;
                    //leftUpperServoPower = UPPER_SERVO_POWER;
                    if(!emptyLeft) {
                        emptyLeft = true;
                        ssom.sorter.emptyLeft();
                    }
                } else if (ssom.gamepadBuffer.g2LeftStickY > ENOUGH_JOYSTICK) {
                    leftIntakeServoPower = 1;
                    //leftUpperServoPower = UPPER_SERVO_POWER;
                } else {
                    leftIntakeServoPower = 0;
                    //leftUpperServoPower = 0;
                    emptyLeft = false;
                }

                // Right intake (outtake wins)
                if (ssom.gamepadBuffer.g2RightStickX > ENOUGH_JOYSTICK || ssom.gamepadBuffer.g2RightStickY > ENOUGH_JOYSTICK) {
                    rightIntakeServoPower = -LOWER_SERVO_POWER;
                    rightUpperServoPower = -UPPER_SERVO_POWER;
                } else if (ssom.gamepadBuffer.g2LeftStickX > ENOUGH_JOYSTICK) {
                    rightIntakeServoPower = LOWER_SERVO_POWER;
                    rightUpperServoPower = UPPER_SERVO_POWER;
                    if(!emptyRight) {
                        emptyRight = true;
                        ssom.sorter.emptyRight();
                    }
                }  else if (ssom.gamepadBuffer.g2LeftStickY > ENOUGH_JOYSTICK) {
                    rightIntakeServoPower = LOWER_SERVO_POWER;
                    rightUpperServoPower = UPPER_SERVO_POWER;
                } else {
                    rightIntakeServoPower = 0;
                    rightUpperServoPower = 0;
                    emptyRight = false;
                }

                // Forward intake (outtake wins)
                if (ssom.gamepadBuffer.g2RightStickY < -ENOUGH_JOYSTICK || ssom.gamepadBuffer.g2RightStickY > ENOUGH_JOYSTICK) {
                    frontIntakeServoPower = -1;
                    //frontUpperServoPower = -UPPER_SERVO_POWER;
                    ssom.sorter.frontAutoTurnOff();
                } else if (ssom.gamepadBuffer.g2LeftStickY < -ENOUGH_JOYSTICK) {
                    frontIntakeServoPower = 1;
                    //frontUpperServoPower = UPPER_SERVO_POWER;
                    if(!emptyFront) {
                        emptyFront = true;
                        ssom.sorter.emptyFront();
                    }
                    ssom.sorter.frontAutoTurnOn();
                } else if (ssom.gamepadBuffer.g2LeftStickY > ENOUGH_JOYSTICK) {
                    frontIntakeServoPower = 1;
                    //frontUpperServoPower = UPPER_SERVO_POWER;
                    ssom.sorter.frontAutoTurnOn();
                } else {
                    frontIntakeServoPower = 0;
                    //frontUpperServoPower = 0;
                    emptyFront = false;
                    ssom.sorter.frontAutoTurnOff();
                }
            }

            frontIntakeServo.setPower(frontIntakeServoPower);
            rightIntakeServo.setPower(rightIntakeServoPower);
            leftIntakeServo.setPower(leftIntakeServoPower);
            //frontUpperServo.setPower(frontUpperServoPower);
            rightUpperServo.setPower(rightUpperServoPower);
            //leftUpperServo.setPower(leftUpperServoPower);

            // Use traditional sleep to not saturate with this thread
            sleep();
        }

        // Cleanup
        frontIntakeStop();
        leftIntakeStop();
        rightIntakeStop();
    }

    @Override
    protected void setTo(IntakeMetric metric) {

    }

    @Override
    public void safeHold() {

    }

    @Override
    protected boolean closeEnough(IntakeMetric metric) {
        return false;
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        if((DEBUG & 16) != 0) {
            telemetry.addData("RightServoPower", rightIntakeServoPower);
            telemetry.addData("LeftServoPower", leftIntakeServoPower);
            telemetry.addData("FrontServoPower", frontIntakeServoPower);
            telemetry.addData("RightUpperServoPower", rightUpperServoPower);
            //telemetry.addData("LeftUpperServoPower", leftUpperServoPower);
            //telemetry.addData("FrontUpperServoPower", frontUpperServoPower);
        }
    }

    /**
     * Turns left intake on
     */

    public void leftIntakeOn() {
        leftIntakeServoPower = 1;
        //leftUpperServoPower = UPPER_SERVO_POWER;
    }

    public boolean isLeftIntakeOn() {
        return leftIntakeServoPower == 1;
    }

    /**
     * Turns right intake on
     */
    public void rightIntakeOn() {
        rightIntakeServoPower = LOWER_SERVO_POWER;
        rightUpperServoPower = UPPER_SERVO_POWER;
    }

    public boolean isRightIntakeOn() {
        return rightIntakeServoPower == LOWER_SERVO_POWER;
    }

    /**
     * Turns on front intake
     */
    public void frontIntakeOn() {
        frontIntakeServoPower = 1;
        //frontUpperServoPower = UPPER_SERVO_POWER;
    }

    public boolean isFrontIntakeOn() {
        return frontIntakeServoPower == 1;
    }

    /**
     * Reverses the intakes
     */
    public void leftIntakeReverse(){
        leftIntakeServoPower = -1;
        //leftUpperServoPower = -UPPER_SERVO_POWER;
    }
    public void rightIntakeReverse(){
        rightIntakeServoPower = -LOWER_SERVO_POWER;
        rightUpperServoPower = -UPPER_SERVO_POWER;
    }
    public void frontIntakeReverse(){
        frontIntakeServoPower = -1;
        //frontUpperServoPower = -UPPER_SERVO_POWER;
    }
    /**
     * Makes intakes stop
     */
    public void leftIntakeStop(){
        leftIntakeServoPower = 0;
        //leftUpperServoPower = 0;
    }
    public void rightIntakeStop(){
        rightIntakeServoPower = 0;
        rightUpperServoPower = 0;
    }
    public void frontIntakeStop(){
        frontIntakeServoPower = 0;
        //frontUpperServoPower = 0;
    }

    // Check the intakes
    public boolean isLeftOff(){
        return leftIntakeServoPower == 0;
    }
    public boolean isRightOff(){
        return rightIntakeServoPower == 0;
    }
    public boolean isFrontOff(){
        return frontIntakeServoPower == 0;
    }
}
