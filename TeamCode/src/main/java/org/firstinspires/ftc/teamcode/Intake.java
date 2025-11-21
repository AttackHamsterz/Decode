package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends RobotPart<IntakeMetric> {
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
        setRunning();
        while (running) {
            if (!ssom.gamepadBuffer.ignoreGamepad) {
                // Left intake (outtake wins)
                if (ssom.gamepadBuffer.g2RightStickX < -0.1 || ssom.gamepadBuffer.g2RightStickY > 0.1) {
                    leftIntakeServoPower = -1;
                } else if (ssom.gamepadBuffer.g2LeftStickX < -0.1 || ssom.gamepadBuffer.g2LeftStickY > 0.1) {
                    leftIntakeServoPower = 1;
                } else {
                    leftIntakeServoPower = 0;
                }

                // Right intake (outtake wins)
                if (ssom.gamepadBuffer.g2RightStickX > 0.1 || ssom.gamepadBuffer.g2RightStickY > 0.1) {
                    rightIntakeServoPower = -1;
                } else if (ssom.gamepadBuffer.g2LeftStickX > 0.1 || ssom.gamepadBuffer.g2LeftStickY > 0.1) {
                    rightIntakeServoPower = 1;
                } else {
                    rightIntakeServoPower = 0;
                }

                // Forward intake (outtake wins)
                if (ssom.gamepadBuffer.g2RightStickY < -0.1 || ssom.gamepadBuffer.g2RightStickY > 0.1) {
                    frontIntakeServoPower = -1;
                } else if (ssom.gamepadBuffer.g2LeftStickY < -0.1 || ssom.gamepadBuffer.g2LeftStickY > 0.1) {
                    frontIntakeServoPower = 1;
                } else {
                    frontIntakeServoPower = 0;
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
}
