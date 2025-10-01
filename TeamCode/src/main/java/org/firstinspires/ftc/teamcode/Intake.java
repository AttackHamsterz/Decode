package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
        this.gamepad = ssom.gamepad2;

        leftIntakeServo = ssom.hardwareMap.get(CRServo.class, "leftIntakeServo"); //need to define channel
        rightIntakeServo = ssom.hardwareMap.get(CRServo.class, "rightIntakeServo"); //need to define channel
        frontIntakeServo = ssom.hardwareMap.get(CRServo.class, "frontIntakeServo"); //need to define channel

        leftIntakeServo.setDirection(CRServo.Direction.REVERSE);
        rightIntakeServo.setDirection(CRServo.Direction.REVERSE);
        frontIntakeServo.setDirection(CRServo.Direction.REVERSE);

        leftIntakeServoPower = 0;
        rightIntakeServoPower = 0;
        frontIntakeServoPower = 0;

    }

    @Override
    public void run() {
        boolean dpadPress = false;
        while (!isInterrupted()) {
            if (!ignoreGamepad) {
                if (!dpadPress) {
                    if (gamepad.dpad_down) {
                        dpadPress = true;
                        leftIntakeServo.setPower(1);
                        leftIntakeServoPower = 1;
                        rightIntakeServo.setPower(1);
                        rightIntakeServoPower = 1;
                        frontIntakeServo.setPower(1);
                        frontIntakeServoPower = 1;
                    }

                    if (gamepad.dpad_left) {
                        dpadPress = true;
                        leftIntakeServo.setPower(1);
                        leftIntakeServoPower = 1;
                    }

                    if (gamepad.dpad_up) {
                        dpadPress = true;
                        frontIntakeServo.setPower(1);
                        frontIntakeServoPower = 1;
                    }

                    if (gamepad.dpad_right) {
                        dpadPress = true;
                        rightIntakeServo.setPower(1);
                        rightIntakeServoPower = 1;
                    }
                }
                else if (!gamepad.dpad_up && !gamepad.dpad_right && !gamepad.dpad_down && !gamepad.dpad_left) {
                    dpadPress = false;
                leftIntakeServo.setPower(0);
                leftIntakeServoPower = 0;
                rightIntakeServo.setPower(0);
                rightIntakeServoPower = 0;
                frontIntakeServo.setPower(0);
                frontIntakeServoPower = 0;}
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
