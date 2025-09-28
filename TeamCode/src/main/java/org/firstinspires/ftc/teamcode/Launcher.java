package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher extends RobotPart<LauncherMetric>{
    private final DcMotor launchMotor;
    private double launchMotorPower;

    public Launcher(StandardSetupOpMode ssom){
        this.ssom = ssom;
        this.gamepad = ssom.gamepad2;
        launchMotor = ssom.hardwareMap.get(DcMotor.class, "launchMotor"); //need to define channel
        // Setup
        launchMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setTargetPosition(0);
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotorPower = 0;
    }

    @Override
    public void run() {
        boolean dpadPress = false;
        while (!isInterrupted()) {
            if (!ignoreGamepad) {
                if (!dpadPress) {
                    if (gamepad.dpad_down) {
                        dpadPress = true;
                        launchMotor.setPower(0);
                        launchMotorPower = 0;
                        //launchMotor.setTargetPosition(1425); //1425.1
                    } else if (gamepad.dpad_up) {
                        dpadPress = true;
                        launchMotor.setPower(1);
                        launchMotorPower = 1;
                        //launchMotor.setTargetPosition(0);
                    } else if (gamepad.dpad_left) {
                        dpadPress = true;
                        if (launchMotorPower > 0.05) {
                            launchMotorPower = launchMotorPower - 0.05;
                            launchMotor.setPower(launchMotorPower);
                        } else {
                            launchMotorPower = 0;
                        }
                    } else if (gamepad.dpad_right) {
                        dpadPress = true;
                        if (launchMotorPower < 1) {
                            launchMotorPower = launchMotorPower + 0.05;
                            launchMotor.setPower(launchMotorPower);
                        }
                    }
                } else if (!gamepad.dpad_up && !gamepad.dpad_right && !gamepad.dpad_down && !gamepad.dpad_left) {
                    dpadPress = false;
                }
            }
        }
    }

    @Override
    protected void setTo(LauncherMetric metric) {

    }

    @Override
    public void safeHold() {

    }

    @Override
    
    
    protected boolean closeEnough(LauncherMetric metric) {
        return false;
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        telemetry.addData("launchMotorPower", launchMotorPower);
    }
}
