package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class RobotPart extends Thread{
    // Opmode reference
    protected StandardSetupOpMode ssom;

    // Gamepad variables
    protected Gamepad gamepad;
    protected Gamepad extraGamepad;
    protected boolean ignoreGamepad = false;

    // Motor overload protection
    protected static final long MOTOR_CHECK_PERIOD_MS = 100;  // Check 10 times a second
    protected static final int CLOSE_ENOUGH_TICKS = 20; // Turn off the other motor when we are close
    protected Thread protectionThread = new Thread();

    // Loop saturation protection
    protected static final long LOOP_PAUSE_MS = 50;

    private class TimeoutThread extends Thread{
        private final int position;

        public TimeoutThread(int position){
            this.position = position;
        }

        @Override
        public void run(){
            try{
                while(Math.abs(getCurrentPosition()-position) > CLOSE_ENOUGH_TICKS) {
                    sleep(LOOP_PAUSE_MS);
                }
                safeHold(getCurrentPosition());
            } catch (InterruptedException ignored) {
            }
        }
    }

    // Force implementing classes to implement the run class
    // This implements the things this body part can do in parallel with
    // other body parts
    @Override
    public abstract void run();

    /**
     * This method should put all motors in this body part into a safe hold mode that uses low
     * power.  It's up to each body part to do that for itself.
     * @param position Tasked position for use in the setHold method
     */
    public abstract void safeHold(int position);

    /**
     * Get the current body part position for use in safe-ing motors.
     * @return the position in ticks
     */
    public abstract int getCurrentPosition();

    /**
     * Display relevant telemetry
     * @param telemetry
     */
    public abstract void getTelemetry(Telemetry telemetry);

    /**
     * Allows a user to toggle using the gamepad
     * @param ignoreGamepad true to ignore gamepad inputs
     */
    public void setIgnoreGamepad(boolean ignoreGamepad)
    {
        this.ignoreGamepad = ignoreGamepad;
    }

    /**
     * This sets up motor protection to avoid overloading motors.  It needs the expected position.
     * Be careful, if we can never get to the target position this will never safe!
     */
    protected void protectMotors(int targetPosition)
    {
        // Cancel old thread
        protectionThread.interrupt();

        // Start a thread that performs safe hold when the time is right
        protectionThread = new TimeoutThread(targetPosition);
        protectionThread.start();
    }
}

