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
    private static final double MAX_LIFT_HEIGHT_LEFT_IN = 21.0;
    private static final double MAX_LIFT_HEIGHT_RIGHT_IN = 21.5;
    private static final double LIFT_HEIGHT_INCREMENT_LEFT_IN = MAX_LIFT_HEIGHT_LEFT_IN / 5.0;
    private static final double LIFT_HEIGHT_INCREMENT_RIGHT_IN = MAX_LIFT_HEIGHT_RIGHT_IN / 5.0;
    private static final double LIFT_HEIGHT_LEFT_START_IN = MAX_LIFT_HEIGHT_LEFT_IN * 0.9;
    private static final double LIFT_HEIGHT_RIGHT_START_IN = MAX_LIFT_HEIGHT_RIGHT_IN * 0.9;
    private static final double LIFT_POWER_LEFT = 0.96;
    private static final double LIFT_POWER_RIGHT = 1.0;
    private static final double DRAG_HEIGHT_INCREMENT_LEFT_IN = 0.45;
    private static final double DRAG_HEIGHT_INCREMENT_RIGHT_IN = 0.45;
    private static final double HOLD_POWER = 0.08;
    private static final double TRIGGER_THRESH = 0.05;
    private static final double PPM_CLOSE = 50;

    private double leftLiftInches = 0;
    private double rightLiftInches = 0;
    private int leftPos = 0;
    private int rightPos = 0;

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

    private void setHeight(double leftInches, double rightInches){
        leftLiftInches = Range.clip(leftInches, 0, MAX_LIFT_HEIGHT_LEFT_IN);
        rightLiftInches = Range.clip(rightInches, 0, MAX_LIFT_HEIGHT_RIGHT_IN);
        leftPos = (int)Math.round(leftLiftInches/PPM_IN*PPM);
        rightPos = (int)Math.round(rightLiftInches/PPM_IN*PPM);
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
                        setHeight(LIFT_HEIGHT_LEFT_START_IN, LIFT_HEIGHT_RIGHT_START_IN);
                    } else if (ssom.gamepadBuffer.g1DpadDown) {
                        pressed = true;
                        //setHeight(0, 0);
                    } else if (ssom.gamepadBuffer.g1DpadRight){
                        pressed = true;
                        setHeight(leftLiftInches + LIFT_HEIGHT_INCREMENT_LEFT_IN, rightLiftInches + LIFT_HEIGHT_INCREMENT_RIGHT_IN);
                    } else if (ssom.gamepadBuffer.g1DpadLeft){
                        pressed = true;
                        //setHeight(leftLiftInches - LIFT_HEIGHT_INCREMENT_LEFT_IN, rightLiftInches - LIFT_HEIGHT_INCREMENT_RIGHT_IN);
                    }
                }
                if (!braked && ssom.gamepadBuffer.g1RightTrigger > TRIGGER_THRESH) {
                    setHeight(DRAG_HEIGHT_INCREMENT_LEFT_IN, DRAG_HEIGHT_INCREMENT_RIGHT_IN);
                    braked = true;
                }
                if (braked && ssom.gamepadBuffer.g1RightTrigger < TRIGGER_THRESH) {
                    setHeight(0, 0);
                    braked = false;
                }

                if(!ssom.gamepadBuffer.g1DpadUp && !ssom.gamepadBuffer.g1DpadDown && !ssom.gamepadBuffer.g1DpadLeft && !ssom.gamepadBuffer.g1DpadRight)
                    pressed = false;

                if(leftPos == 0 && Math.abs(finalLiftMotorLeft.getCurrentPosition()) < PPM_CLOSE){
                    finalLiftMotorLeft.setPower(HOLD_POWER);
                    finalLiftMotorRight.setPower(HOLD_POWER);
                }
                else{
                    finalLiftMotorLeft.setTargetPosition(leftPos);
                    finalLiftMotorRight.setTargetPosition(rightPos);
                    finalLiftMotorLeft.setPower(LIFT_POWER_LEFT);
                    finalLiftMotorRight.setPower(LIFT_POWER_RIGHT);
                }

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


