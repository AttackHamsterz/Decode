package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

@Autonomous(name = "Robot Setup Super Class", group = "Robot")
@Disabled
public class StandardSetupOpMode extends OpMode {
    protected static final double AUTO_MOVE_POWER = 1.0;

    public enum COLOR {
        RED,
        BLUE
    }
    public enum POSITION {
        BOARD,
        FIELD
    }
    public static String colorToString(COLOR color)
    {
        return (color == COLOR.RED) ? "Red" : "Blue";
    }
    public static String positionToString(POSITION pos)
    {
        return (pos == POSITION.BOARD) ? "Board" : "Field";
    }

    // Team objects
    public COLOR color = COLOR.BLUE;
    public POSITION position = POSITION.FIELD;
    public boolean ignoreGamepad = false;

    // Timing
    private Timer opmodeTimer;

    // Robot Objects
    public FinalLift finalLift;
    public Motion motion;
    public Sorter sorter;
    public BallLifter ballLifter;
    public Launcher launcher;
    public Intake intake;

    // Telemetry
    static TelemetryManager telemetryM;

    @Override public void init() {
        // Setup telemetry
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Setup robot
        motion = new Motion(this, ignoreGamepad);
        sorter = new Sorter(this, ignoreGamepad);
        ballLifter = new BallLifter(this, ignoreGamepad);
        finalLift = new FinalLift(this, ignoreGamepad);
        launcher = new Launcher(this);
        intake = new Intake(this);

        // Setup timing
        opmodeTimer = new Timer();

        // Let them now we are initialized
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Color", colorToString(color));
        if(ignoreGamepad)
            telemetry.addData("Auto Position", positionToString(position));
        else
            telemetry.addLine("Teleop Ready!");
        telemetry.update();
    }
    @Override public void init_loop() {
        // If autonomous
        if (ignoreGamepad) {
            // Watch obolisk for april tag
        }
    }
    @Override public void start() {
        // Reset timer and launch Threads
        opmodeTimer.resetTimer();
        motion.start();
        sorter.start();
        ballLifter.start();
        finalLift.start();
        launcher.start();
        intake.start();
    }
    @Override public void loop() { }
    @Override public void stop() {
        try {
            waitForCompletion();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
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
        sorter.interrupt();
        ballLifter.interrupt();
        finalLift.interrupt();
        launcher.interrupt();

        // Wait for threads to complete
        motion.join();
    }
}
