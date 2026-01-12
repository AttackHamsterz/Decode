package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FinalLift extends RobotPart<FinalLiftMetric>{

    private final DcMotor finalLiftMotorLeft;
    private final DcMotor finalLiftMotorRight;
    private static final double PPM = 1993.6;
    private static final double PPM_IN = 4.75;
    private static final double MAX_LIFT_HEIGHT_LEFT_IN = 20.0;
    private static final double MAX_LIFT_HEIGHT_RIGHT_IN = 20.5;
    private static final double LIFT_HEIGHT_INCREMENT_LEFT_IN = MAX_LIFT_HEIGHT_LEFT_IN / 5.0;
    private static final double LIFT_HEIGHT_INCREMENT_RIGHT_IN = MAX_LIFT_HEIGHT_RIGHT_IN / 5.0;
    private static final double LIFT_HEIGHT_LEFT_START_IN = MAX_LIFT_HEIGHT_LEFT_IN * 0.9;
    private static final double LIFT_HEIGHT_RIGHT_START_IN = MAX_LIFT_HEIGHT_RIGHT_IN * 0.9;
    private static final double LIFT_POWER_LEFT = 0.97;
    private static final double LIFT_POWER_RIGHT = 1.0;
    private static final double DRAG_HEIGHT_INCREMENT_LEFT_IN = 0.45;
    private static final double DRAG_HEIGHT_INCREMENT_RIGHT_IN = 0.45;
    private static final double HOLD_POWER = 0.1;
    private static final double TRIGGER_THRESH = 0.05;

    private double leftLiftInches = 0;
    private double rightLiftInches = 0;

    public FinalLift(StandardSetupOpMode ssom){
        this.ssom = ssom;
        finalLiftMotorLeft = ssom.hardwareMap.get(DcMotor.class, "finalLiftMotorLeft");
        finalLiftMotorRight = ssom.hardwareMap.get(DcMotor.class, "finalLiftMotorRight");
        finalLiftMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        finalLiftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        finalLiftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        finalLiftMotorLeft.setTargetPosition(0);
        finalLiftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        finalLiftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        finalLiftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        finalLiftMotorRight.setTargetPosition(0);
        finalLiftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void lift() {
        leftLiftInches = LIFT_HEIGHT_LEFT_START_IN;
        rightLiftInches = LIFT_HEIGHT_RIGHT_START_IN;
        int leftPos = (int)Math.round(LIFT_HEIGHT_LEFT_START_IN/PPM_IN*PPM);
        int rightPos = (int)Math.round(LIFT_HEIGHT_RIGHT_START_IN/PPM_IN*PPM);
        finalLiftMotorLeft.setTargetPosition(leftPos);
        finalLiftMotorRight.setTargetPosition(rightPos);
        finalLiftMotorLeft.setPower(LIFT_POWER_LEFT);
        finalLiftMotorRight.setPower(LIFT_POWER_RIGHT);
    }

    public void drop() {
        leftLiftInches = 0;
        rightLiftInches = 0;
        finalLiftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        finalLiftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        finalLiftMotorLeft.setPower(0);
        finalLiftMotorRight.setPower(0);
    }

    public void slightLift() {
        leftLiftInches = Range.clip(leftLiftInches + LIFT_HEIGHT_INCREMENT_LEFT_IN, 0, MAX_LIFT_HEIGHT_LEFT_IN);
        rightLiftInches = Range.clip(rightLiftInches + LIFT_HEIGHT_INCREMENT_RIGHT_IN, 0, MAX_LIFT_HEIGHT_RIGHT_IN);
        int leftPos = (int)Math.round(leftLiftInches/PPM_IN*PPM);
        int rightPos = (int)Math.round(rightLiftInches/PPM_IN*PPM);
        finalLiftMotorLeft.setTargetPosition(leftPos);
        finalLiftMotorRight.setTargetPosition(rightPos);
        finalLiftMotorLeft.setPower(LIFT_POWER_LEFT);
        finalLiftMotorRight.setPower(LIFT_POWER_RIGHT);
    }

    public void slightDrop() {
        leftLiftInches = Range.clip(leftLiftInches - LIFT_HEIGHT_INCREMENT_LEFT_IN, 0, MAX_LIFT_HEIGHT_LEFT_IN);
        rightLiftInches = Range.clip(rightLiftInches - LIFT_HEIGHT_INCREMENT_RIGHT_IN, 0, MAX_LIFT_HEIGHT_RIGHT_IN);
        int leftPos = (int)Math.round(leftLiftInches/PPM_IN*PPM);
        int rightPos = (int)Math.round(rightLiftInches/PPM_IN*PPM);
        finalLiftMotorLeft.setTargetPosition(leftPos);
        finalLiftMotorRight.setTargetPosition(rightPos);
        finalLiftMotorLeft.setPower(HOLD_POWER);
        finalLiftMotorRight.setPower(HOLD_POWER);
    }

    public void brakeDrop() {
        leftLiftInches = DRAG_HEIGHT_INCREMENT_LEFT_IN;
        rightLiftInches = DRAG_HEIGHT_INCREMENT_RIGHT_IN;
        int leftPos = (int)Math.round(DRAG_HEIGHT_INCREMENT_LEFT_IN/PPM_IN*PPM);
        int rightPos = (int)Math.round(DRAG_HEIGHT_INCREMENT_RIGHT_IN/PPM_IN*PPM);
        finalLiftMotorLeft.setTargetPosition(leftPos);
        finalLiftMotorRight.setTargetPosition(rightPos);
        finalLiftMotorLeft.setPower(LIFT_POWER_LEFT);
        finalLiftMotorRight.setPower(LIFT_POWER_RIGHT);
    }

    public void initialPosition() {
        int leftPos = 0;
        int rightPos = 0;
        finalLiftMotorLeft.setTargetPosition(leftPos);
        finalLiftMotorRight.setTargetPosition(rightPos);
        finalLiftMotorLeft.setPower(HOLD_POWER);
        finalLiftMotorRight.setPower(HOLD_POWER);
    }

    @Override
    public void run() {
        if (!ssom.gamepadBuffer.ignoreGamepad) {
            setRunning();
            boolean pressed = false;
            boolean braked = false;
            while (running){
                if (ssom.gamepadBuffer.g1Start && !pressed){
                    if(ssom.gamepadBuffer.g1DpadUp) {
                        pressed = true;
                        lift();
                    } else if (ssom.gamepadBuffer.g1DpadDown) {
                        pressed = true;
                        drop();
                    } else if (ssom.gamepadBuffer.g1DpadRight){
                        pressed = true;
                        slightLift ();
                    } else if (ssom.gamepadBuffer.g1DpadLeft){
                        pressed = true;
                        slightDrop ();
                    }
                }
                if (!braked && ssom.gamepadBuffer.g1RightTrigger > TRIGGER_THRESH) {
                        brakeDrop();
                        braked = true;

                }
                if (braked && ssom.gamepadBuffer.g1RightTrigger < TRIGGER_THRESH) {
                    initialPosition();
                    braked = false;
                }

                if(!ssom.gamepadBuffer.g1DpadUp && !ssom.gamepadBuffer.g1DpadDown && !ssom.gamepadBuffer.g1DpadLeft && !ssom.gamepadBuffer.g1DpadRight)
                    pressed = false;

                // Short sleep to keep this loop from saturating
                sleep();
            }
        }
    }

    @Override
    protected void setTo(FinalLiftMetric metric) {

    }

    @Override
    public void safeHold() {

    }

    @Override
    protected boolean closeEnough(FinalLiftMetric metric) {
        return false;
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        if((DEBUG & 64) != 0) {
            telemetry.addData("liftLeftTicks", finalLiftMotorLeft.getCurrentPosition());
            telemetry.addData("liftRightTicks", finalLiftMotorRight.getCurrentPosition());
            telemetry.addData("liftLeftPower", finalLiftMotorLeft.getPower());
            telemetry.addData("liftRightPower", finalLiftMotorRight.getPower());
        }
    }
}


