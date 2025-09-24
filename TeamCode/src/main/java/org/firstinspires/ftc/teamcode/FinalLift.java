package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FinalLift extends RobotPart<FinalLiftMetric>{

    private final DcMotor finalLiftMotorLeft;
    private final DcMotor finalLiftMotorRight;
    private static final double PPM = 1993.6;
    private static final double PPM_IN = 4.75;
    private static final double LIFTHEIGHT_IN = 18.5;

    public FinalLift(StandardSetupOpMode ssom, boolean ignoreGamepad){
        this.ssom = ssom;
        this.gamepad = ssom.gamepad1;
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
        int pos = (int)Math.round(LIFTHEIGHT_IN/PPM_IN*PPM);
        finalLiftMotorLeft.setTargetPosition(pos);
        finalLiftMotorRight.setTargetPosition(pos);
        finalLiftMotorLeft.setPower(1);
        finalLiftMotorRight.setPower(1);
    }

    public void drop() {
        int pos = 0;
        finalLiftMotorLeft.setTargetPosition(pos);
        finalLiftMotorRight.setTargetPosition(pos);
        finalLiftMotorLeft.setPower(1);
        finalLiftMotorRight.setPower(1);
    }

    @Override
    public void run() {
        while (!isInterrupted()){
                if (!ignoreGamepad) {
                    if (gamepad.start) {
                        lift();
                    } else if (gamepad.back) {
                        drop();

                    }
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
        telemetry.addData("liftLeftTicks", finalLiftMotorLeft.getCurrentPosition());
        telemetry.addData("liftRightTicks", finalLiftMotorRight.getCurrentPosition());
    }
}


