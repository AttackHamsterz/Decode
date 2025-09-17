package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Sorter extends RobotPart<SorterMetric>{
    private final DcMotor sortMotor;
    public Sorter(StandardSetupOpMode ssom, boolean ignoreGamepad){
        this.ssom = ssom;
        this.gamepad = ssom.gamepad2;
        this.ignoreGamepad = ignoreGamepad;
        sortMotor = ssom.hardwareMap.get(DcMotor.class, "sortMotor"); //need to define channel
        // Setup
        sortMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        sortMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sortMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sortMotor.setTargetPosition(0);
        sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            if (!ignoreGamepad) {
                if (gamepad.a) {
                    sortMotor.setPower(1);
                    sortMotor.setTargetPosition(670);
                }
                else if (gamepad.b) {
                    sortMotor.setPower(1);
                    sortMotor.setTargetPosition(0);
                }
                else{
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
    }
}

