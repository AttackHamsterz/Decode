package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class to implement the ball lift mechanism.  It's up to other classes to determine if lifting
 * is actually safe!
 */
public class BallLifter extends RobotPart<BallLifterMetric>{
    private final DigitalChannel ballLiftSwitch;
    private final CRServo ballLiftServo;

    private final int INITIAL_WAIT_MS = 150;  // Delay so the magnet moves from the switch (ms)
    private final int LIFT_COMPLETE_MS = 400; // Lift completes and snaps back (ms)
    private final int TOTAL_WAIT_MS = 1000;   // Max time to wait for reset (ms)

    private boolean lifting;

    public BallLifter(StandardSetupOpMode ssom, boolean ignoreGamepad){
        this.ssom = ssom;
        this.setIgnoreGamepad(ignoreGamepad);
        this.lifting = false;

        ballLiftSwitch = ssom.hardwareMap.get(DigitalChannel.class, "ballLiftSwitch");
        ballLiftSwitch.setMode(DigitalChannel.Mode.INPUT);
        ballLiftServo = ssom.hardwareMap.get(CRServo.class, "ballLiftServo");
        ballLiftServo.setDirection(DcMotorSimple.Direction.REVERSE);
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
            ballLiftServo.setPower(1.0);
            long startTimeMs = System.currentTimeMillis();
            boolean done = false;
            while(!done){
                long nowMs = System.currentTimeMillis();
                long deltaT = nowMs - startTimeMs;
                if(deltaT > LIFT_COMPLETE_MS)
                    lifting = false;
                if(deltaT > INITIAL_WAIT_MS && !ballLiftSwitch.getState()){
                    done = true;
                }else if(deltaT > TOTAL_WAIT_MS){
                    done = true;
                }
            }
            ballLiftServo.setPower(0.0);
            lifting = false;
        });
        thread.start();

    }

    @Override
    public void run() {
        boolean pressed = false;
        if (!ignoreGamepad) {
            while (!isInterrupted()) {
                if (!pressed && ssom.gamepadBuffer.g2RightTrigger > 0.1) {
                    pressed = true;
                    lift();
                }

                if (ssom.gamepadBuffer.g2RightTrigger <= 0.1)
                    pressed = false;

                // Short sleep to keep this loop from saturating
                sleep();
            }
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

    }

    public boolean isLifting()
    {
        return lifting;
    }
}
