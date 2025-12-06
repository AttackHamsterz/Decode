package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends RobotPart<IntakeMetric> {
    private static final double ENOUGH_JOYSTICK = 0.1;
    private final CRServo leftIntakeServo;
    private final CRServo frontIntakeServo;
    private final CRServo rightIntakeServo;
    private double leftIntakeServoPower;
    private double rightIntakeServoPower;
    private double frontIntakeServoPower;


    public Intake(StandardSetupOpMode ssom) {

        this.ssom = ssom;
        leftIntakeServo = ssom.hardwareMap.get(CRServo.class, "leftIntakeServo"); //need to define channel
        rightIntakeServo = ssom.hardwareMap.get(CRServo.class, "rightIntakeServo"); //need to define channel
        frontIntakeServo = ssom.hardwareMap.get(CRServo.class, "frontIntakeServo"); //need to define channel

        leftIntakeServo.setDirection(CRServo.Direction.FORWARD);
        rightIntakeServo.setDirection(CRServo.Direction.FORWARD);
        frontIntakeServo.setDirection(CRServo.Direction.FORWARD);

        leftIntakeServoPower = 0;
        rightIntakeServoPower = 0;
        frontIntakeServoPower = 0;

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
                } else if (ssom.gamepadBuffer.g2LeftStickX < -ENOUGH_JOYSTICK) {
                    leftIntakeServoPower = 1;
                    if(!emptyLeft) {
                        emptyLeft = true;
                        ssom.sorter.emptyLeft();
                    }
                } else if (ssom.gamepadBuffer.g2LeftStickY > ENOUGH_JOYSTICK) {
                    leftIntakeServoPower = 1;
                } else {
                    leftIntakeServoPower = 0;
                    emptyLeft = false;
                }

                // Right intake (outtake wins)
                if (ssom.gamepadBuffer.g2RightStickX > ENOUGH_JOYSTICK || ssom.gamepadBuffer.g2RightStickY > ENOUGH_JOYSTICK) {
                    rightIntakeServoPower = -1;
                } else if (ssom.gamepadBuffer.g2LeftStickX > ENOUGH_JOYSTICK) {
                    rightIntakeServoPower = 1;
                    if(!emptyRight) {
                        emptyRight = true;
                        ssom.sorter.emptyRight();
                    }
                }  else if (ssom.gamepadBuffer.g2LeftStickY > ENOUGH_JOYSTICK) {
                    rightIntakeServoPower = 1;
                } else {
                    rightIntakeServoPower = 0;
                    emptyRight = false;
                }

                // Forward intake (outtake wins)
                if (ssom.gamepadBuffer.g2RightStickY < -ENOUGH_JOYSTICK || ssom.gamepadBuffer.g2RightStickY > ENOUGH_JOYSTICK) {
                    frontIntakeServoPower = -1;
                    ssom.sorter.frontAutoTurnOff();
                } else if (ssom.gamepadBuffer.g2LeftStickY < -ENOUGH_JOYSTICK) {
                    frontIntakeServoPower = 1;
                    if(!emptyFront) {
                        emptyFront = true;
                        ssom.sorter.emptyFront();
                    }
                    ssom.sorter.frontAutoTurnOn();
                } else if (ssom.gamepadBuffer.g2LeftStickY > ENOUGH_JOYSTICK) {
                    frontIntakeServoPower = 1;
                    ssom.sorter.frontAutoTurnOn();
                } else {
                    frontIntakeServoPower = 0;
                    emptyFront = false;
                    ssom.sorter.frontAutoTurnOff();
                }
            }

            frontIntakeServo.setPower(frontIntakeServoPower);
            rightIntakeServo.setPower(rightIntakeServoPower);
            leftIntakeServo.setPower(leftIntakeServoPower);

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
        }
    }

    /**
     * Turns left intake on
     */

    public void leftIntakeOn() {
        leftIntakeServoPower = 1;
    }

    /**
     * Turns right intake on
     */
    public void rightIntakeOn() {
        rightIntakeServoPower = 1;
    }

    /**
     * Turns on front intake
     */
    public void frontIntakeOn() {
        frontIntakeServoPower = 1;
    }

    /**
     * Reverses the intakes
     */
    public void leftIntakeReverse(){
        leftIntakeServoPower = -1;
    }
    public void rightIntakeReverse(){
        rightIntakeServoPower = -1;
    }
    public void frontIntakeReverse(){
        frontIntakeServoPower = -1;
    }
    /**
     * Makes intakes stop
     */
    public void leftIntakeStop(){
        leftIntakeServoPower = 0;
    }
    public void rightIntakeStop(){
        rightIntakeServoPower = 0;
    }
    public void frontIntakeStop(){
        frontIntakeServoPower = 0;
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
