package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.util.Timer;

@Autonomous(name = "Robot Setup Super Class", group = "Robot")
@Disabled
public class StandardSetupOpMode extends LinearOpMode {
    protected static final double AUTO_MOVE_POWER = 1.0;

    public enum COLOR {
        RED,
        BLUE
    }
    public enum POSITION {
        BOARD,
        FIELD
    }

    // Team objects
    public COLOR color = COLOR.BLUE;
    public POSITION position = POSITION.FIELD;
    public boolean ignoreGamepad = false;

    // Timing
    private Timer opmodeTimer;

    // Robot Objects
    public Motion motion;

    // @Override
    public void runOpMode() throws InterruptedException {

        // Setup robot
        motion = new Motion(this);
        setIgnoreGamepad(ignoreGamepad);

        // Setup timing
        opmodeTimer = new Timer();

        // Put any required init for autonomous, remember teleop can't move the robot!
        if (ignoreGamepad) {

        }

        // Update status for the user
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        opmodeTimer.resetTimer();

        // Launch Threads
        motion.start();
    }

    /**
     * Method sets up this specific opmode
     * @param color color robot should use (and always yellow)
     * @param position position robot is starting on
     * @param ignoreGamepad true to ignore gamepad input
     */
    protected void setup(COLOR color, POSITION position, boolean ignoreGamepad) {
        // Setup position and color for this opmode
        this.color = color;
        this.position = position;

        // Should we ignore the gamepad or not?
        setIgnoreGamepad(ignoreGamepad);
    }

    /**
     * Disable all gamepads for autonomous
     * @param ignoreGamepad true ignores gamepads
     */
    public void setIgnoreGamepad(boolean ignoreGamepad)
    {
        this.ignoreGamepad = ignoreGamepad;
        if(motion != null) motion.setIgnoreGamepad(ignoreGamepad);
    }

    /**
     * Interrupt all the active threads and wait for them to complete
     * @throws InterruptedException failed to wait for completion
     */
    public void waitForCompletion() throws InterruptedException
    {
        // Interrupt all the running threads
        motion.interrupt();

        // Wait for threads to complete
        motion.join();
    }
}
