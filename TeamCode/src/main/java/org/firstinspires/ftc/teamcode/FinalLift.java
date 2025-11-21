package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FinalLift extends RobotPart<FinalLiftMetric>{

    private final DcMotor finalLiftMotorLeft;
    private final DcMotor finalLiftMotorRight;
    private static final double PPM = 1993.6;
    private static final double PPM_IN = 4.75;
    private static final double LIFT_HEIGHT_IN = 12;

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
        int pos = (int)Math.round(LIFT_HEIGHT_IN/PPM_IN*PPM);
        finalLiftMotorLeft.setTargetPosition(pos);
        finalLiftMotorRight.setTargetPosition(pos);
        finalLiftMotorLeft.setPower(1);
        finalLiftMotorRight.setPower(1);
    }

    public void drop() {
        int pos = 0;
        finalLiftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        finalLiftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        finalLiftMotorLeft.setPower(0);
        finalLiftMotorRight.setPower(0);
    }

    @Override
    public void run() {
        if (!ssom.gamepadBuffer.ignoreGamepad) {
            setRunning();
            while (running){
                if (ssom.gamepadBuffer.g1Start){
                    if(ssom.gamepadBuffer.g1DpadUp) {
                        lift();
                    } else if (ssom.gamepadBuffer.g1DpadDown) {
                        drop();
                    }
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


