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
        if (!ignoreGamepad) {
            while (!isInterrupted()) {
                // Left intake (outtake wins)
                if(ssom.gamepadBuffer.g2RightStickX < -0.1 || ssom.gamepadBuffer.g2RightStickY > 0.1){
                    leftIntakeServo.setPower(-1);
                    leftIntakeServoPower = -1;
                }
                else if(ssom.gamepadBuffer.g2LeftStickX < -0.1 || ssom.gamepadBuffer.g2LeftStickY > 0.1){
                    leftIntakeServo.setPower(1);
                    leftIntakeServoPower = 1;
                }
                else{
                    leftIntakeServo.setPower(0);
                    leftIntakeServoPower = 0;
                }

                // Right intake (outtake wins)
                if(ssom.gamepadBuffer.g2RightStickX > 0.1 || ssom.gamepadBuffer.g2RightStickY > 0.1){
                    rightIntakeServo.setPower(-1);
                    rightIntakeServoPower = -1;
                }
                else if(ssom.gamepadBuffer.g2LeftStickX > 0.1 || ssom.gamepadBuffer.g2LeftStickY > 0.1){
                    rightIntakeServo.setPower(1);
                    rightIntakeServoPower = 1;
                }
                else{
                    rightIntakeServo.setPower(0);
                    rightIntakeServoPower = 0;
                }

                // Forward intake (outtake wins)
                if(ssom.gamepadBuffer.g2RightStickY < -0.1 || ssom.gamepadBuffer.g2RightStickY > 0.1){
                    frontIntakeServo.setPower(-1);
                    frontIntakeServoPower = -1;
                }
                else if(ssom.gamepadBuffer.g2LeftStickY < -0.1 || ssom.gamepadBuffer.g2LeftStickY > 0.1){
                    frontIntakeServo.setPower(1);
                    frontIntakeServoPower = 1;
                }
                else{
                    frontIntakeServo.setPower(0);
                    frontIntakeServoPower = 0;
                }

                // Use traditional sleep to not saturate with this thread
                sleep();
            }
        }
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
        telemetry.addData("RightServoPower", rightIntakeServoPower);
        telemetry.addData("LeftServoPower", leftIntakeServoPower);
        telemetry.addData("FrontServoPower", frontIntakeServoPower);
    }
}
