package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Observer;

public abstract class RobotPart<T> extends Thread{
    // Opmode reference
    protected StandardSetupOpMode ssom;
    protected boolean running;

    // DEBUG drives telemetry output
    // Set bit positions to enable disable outputs
    //     1 - Motion
    //     2 - Launcher
    //     4 - Eye
    //     8 - Sorter
    //    16 - Intake
    //    32 - Ball lifter
    //    64 - Final Lift
    //   So to dump Motion, Sorter and Intake: 0b00011001 -or- 1+8+16
    public static final int DEBUG = 0;

    // Motor overload protection
    protected static final long SAFE_CHECK_PERIOD_MS = 100;  // Check 10 times a second
    protected static final long LOOP_PAUSE_MS = 100;
    protected Thread protectionThread = new Thread();

    private class TimeoutThread extends Thread{
        private final T metric;
        private final Observer observer;

        public TimeoutThread(T metric, Observer observer){
            this.metric = metric;
            this.observer = observer;
        }

        @Override
        public void run(){
            // Wait until our metric is met
            try{
                while(!closeEnough(metric)) {
                    sleep(LOOP_PAUSE_MS);
                    if(isInterrupted())
                        break;
                }
                if(!isInterrupted())
                    safeHold();
            } catch (InterruptedException ignored) {
            }

            // Let the observer know we are done or interrupted
            if(observer != null)
                observer.notify();
        }
    }

    /**
     * Force implementing classes to implement the run class.
     * This implements the things this body part can do in parallel with
     * other body parts.
     */
    @Override
    public abstract void run();

    /**
     * Forces the body part to set itself to the provided metric
     * @param metric value to set our body part to
     */
    protected abstract void setTo(T metric);

    /**
     * This method should put all motors/servos in this body part into a safe mode that uses low
     * power.  It's up to each body part to do that for itself.
     */
    public abstract void safeHold();

    /**
     * Force implementing classes to check if a metric is close to another metric.
     * Metric could be (inches, ticks, time, a task, etc.)
     * @return true if we're close enough to consider the task done, false if not
     */
    protected abstract boolean closeEnough(T metric);

    /**
     * This runs the body part to hit a metric.  We also setup a protection thread
     * to avoid overloading motors and servos.
     * @param targetMetric expected metric (position, power, time, etc.).
     * @param observer optional observer to notify when we are done
     * Be careful, if we can never get to the target position this will never safe!
     */
    public void set(T targetMetric, Observer observer)
    {
        // Cancel old thread
        protectionThread.interrupt();

        // Start a thread that performs safe hold when the time is right
        protectionThread = new TimeoutThread(targetMetric, observer);
        setTo(targetMetric);
        protectionThread.start();
    }

    /**
     * Display relevant telemetry to the user for debugging
     * @param telemetry place to put the info
     */
    public abstract void getTelemetry(Telemetry telemetry);

    /**
     * Tell the robot part we are running threads
     */
    public void setRunning(){
        running = true;
    }

    /**
     * Signal any robot part threads to stop and cleanup
     */
    public void safeStop(){
        running = false;
    }

    /**
     * Traditional sleep for a robot part (to avoid saturating threads)
     */
    public void sleep(){
        try{
            sleep(LOOP_PAUSE_MS);
        } catch (InterruptedException ignored) {
        }
    }
}

