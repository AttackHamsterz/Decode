package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class to implement the ball lift mechanism.  It's up to other classes to determine if lifting
 * is actually safe!
 */
public class BallLifter extends RobotPart<BallLifterMetric>{
    private final DigitalChannel ballLiftSwitch;
    private final CRServo ballLiftServo;
    private final LED redLED;
    private final LED greenLED;

    private static final int INITIAL_WAIT_MS = 150;    // Delay so the magnet moves from the switch (ms)
    private static final int LIFT_COMPLETE_MS = 1500;   // Lift completes and snaps back (ms)
    private static final int TOTAL_WAIT_MS = 1500;     // Max time to wait for reset (ms)
    private static final double TRIGGER_THRESH = 0.02; // How much trigger triggers action
    private static final double LIFT_POWER = 1.0;
    private static final double STALL_POWER = 0.0;

    private boolean lifting;
    private boolean reset;

    public BallLifter(StandardSetupOpMode ssom){
        this.ssom = ssom;
        this.lifting = false;
        this.reset = true;

        ballLiftSwitch = ssom.hardwareMap.get(DigitalChannel.class, "ballLiftSwitch");
        ballLiftSwitch.setMode(DigitalChannel.Mode.INPUT);
        ballLiftServo = ssom.hardwareMap.get(CRServo.class, "ballLiftServo");
        ballLiftServo.setDirection(DcMotorSimple.Direction.REVERSE);

        redLED = ssom.hardwareMap.get(LED.class, "redLED");
        greenLED = ssom.hardwareMap.get(LED.class, "greenLED");
        redLED.off();
        greenLED.off();
    }

    /**
     * This method will run the lift servo until zeroed again.  This method will return right
     * away so be careful!
     */
    public void lift(){
        // Lifting thread
        Thread thread = new Thread(() -> {
            // TODO - Make sure sorter isn't moving

            // Start the lift
            lifting = true;
            ballLiftServo.setPower(LIFT_POWER);
            long startTimeMs = System.currentTimeMillis();
            reset = false;
            while(!reset){
                long nowMs = System.currentTimeMillis();
                long deltaT = nowMs - startTimeMs;
                if(deltaT > LIFT_COMPLETE_MS)
                    lifting = false;
                if(deltaT > INITIAL_WAIT_MS && !ballLiftSwitch.getState()){
                    reset = true;
                }else if(deltaT > TOTAL_WAIT_MS){
                    reset = true;
                }
            }
            ballLiftServo.setPower(STALL_POWER);
            lifting = false;
        });
        thread.start();

    }

    @Override
    public void run() {
        boolean pressed = false;
        setRunning();
        while (running) {
            if (!ssom.gamepadBuffer.ignoreGamepad) {
                if (!pressed && ssom.gamepadBuffer.g2RightTrigger > TRIGGER_THRESH) {
                    pressed = true;
                    lift();
                }

                if (ssom.gamepadBuffer.g2RightTrigger <= TRIGGER_THRESH)
                    pressed = false;
            }

            // Color indication
            if(ssom.launcher.launchReady() && ssom.eye.linedUp() && !ssom.sorter.isSpinning() && isReset()){
                redLED.off();
                greenLED.on();
            }
            else{
                redLED.on();
                greenLED.off();
            }

            // Short sleep to keep this loop from saturating
            sleep();
        }
    }

    @Override
    protected void setTo(BallLifterMetric metric) {

    }

    @Override
    public void safeHold() {

    }

    @Override
    protected boolean closeEnough(BallLifterMetric metric) {
        return false;
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        if((DEBUG & 32) != 0) {
        }
    }

    public boolean isLifting(){
        return lifting;
    }

    public boolean isReset(){
        return reset;
    }
}
