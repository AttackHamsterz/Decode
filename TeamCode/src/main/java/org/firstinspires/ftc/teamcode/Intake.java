package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends RobotPart<IntakeMetric> {
    private final DigitalChannel leftIntakeSwitch;
    private final DigitalChannel rightIntakeSwitch;
    private final DigitalChannel frontIntakeSwitch;
    private final CRServo leftIntake;
    private final CRServo rightIntake;
    private final CRServo frontIntake;
    private final int INITIAL_WAIT_MS = 150;  // Delay so the magnet moves from the switch (ms)
    private final int TOTAL_WAIT_MS = 1000;   // Max time to wait for reset (ms)
    private Thread stopThread;

    public Intake(StandardSetupOpMode ssom, boolean ignoreGamepad){
        this.ssom = ssom;
        this.gamepad = ssom.gamepad1;
        this.ignoreGamepad = ignoreGamepad;
        leftIntakeSwitch = ssom.hardwareMap.get(DigitalChannel.class, "ballLiftSwitch");
        leftIntakeSwitch.setMode(DigitalChannel.Mode.INPUT);
        rightIntakeSwitch = ssom.hardwareMap.get(DigitalChannel.class, "ballLiftSwitch");
        rightIntakeSwitch.setMode(DigitalChannel.Mode.INPUT);
        frontIntakeSwitch = ssom.hardwareMap.get(DigitalChannel.class, "ballLiftSwitch");
        frontIntakeSwitch.setMode(DigitalChannel.Mode.INPUT);
        this.leftIntake = ssom.hardwareMap.get(CRServo.class, "leftIntake"); //ch0 expansion hub Servo
        this.rightIntake = ssom.hardwareMap.get(CRServo.class, "rightIntake"); //ch1 expansion hub Continuous Servo
        this.frontIntake = ssom.hardwareMap.get(CRServo.class, "frontIntake"); //ch2 expansion hub Continuous Servo
        // Setup
        this.leftIntake.setDirection(CRServo.Direction.REVERSE);
        this.rightIntake.setDirection(CRServo.Direction.REVERSE);
        this.frontIntake.setDirection(CRServo.Direction.REVERSE);
        this.stopThread = new Thread();
    }

    /**
     * This method will run the lift servo until zeroed again.  This method will return right
     * away so be careful!
     */
    public void lift(CRServo intakeServo, DigitalChannel intakeSwitch){
        // Start servo
        intakeServo.setPower(1.0);

        // Start thread for stop
        Thread thread = new Thread(() -> {
            long startTimeMs = System.currentTimeMillis();
            boolean done = false;
            while(!done){
                long nowMs = System.currentTimeMillis();
                long deltaT = nowMs - startTimeMs;
                if(deltaT > TOTAL_WAIT_MS){
                    done = true;
                }
                else if(deltaT > INITIAL_WAIT_MS && !intakeSwitch.getState()){
                    done = true;
                }
            }
            intakeServo.setPower(0.0);
        });
        thread.start();
    }

    @Override
    public void run() {
        boolean dpadUpPressed = false;
        boolean dpadLeftPressed = false;
        boolean dpadRightPressed = false;
        while (!isInterrupted()) {
            if (!ignoreGamepad) {
                if (!dpadUpPressed && gamepad.dpad_up) {
                    dpadUpPressed = true;
                    lift(frontIntake, frontIntakeSwitch);
                } else if (!dpadRightPressed && gamepad.dpad_right) {
                    dpadRightPressed = true;
                    lift(rightIntake, rightIntakeSwitch);
                } else if (!dpadLeftPressed && gamepad.dpad_left) {
                    dpadLeftPressed = true;
                    lift(leftIntake, leftIntakeSwitch);
                }

                if(!gamepad.dpad_up)
                    dpadUpPressed = false;
                if(!gamepad.dpad_right)
                    dpadRightPressed = false;
                if(!gamepad.dpad_left)
                    dpadLeftPressed = false;
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
        //telemetry.addData("sorterticks", sortMotor.getCurrentPosition());
    }
}

