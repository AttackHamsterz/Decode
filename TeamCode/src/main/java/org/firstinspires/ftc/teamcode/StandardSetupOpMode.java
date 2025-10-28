package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import java.util.ArrayList;

@Autonomous(name = "Robot Setup Super Class", group = "Robot")
@Disabled
public class StandardSetupOpMode extends OpMode {
    protected static final double AUTO_MOVE_POWER = 1.0;
    protected static final long GAMEPAD_POLLING_INTERVAL_MS = 10;

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

    // Gamepad buffers
    public GamepadBuffer gamepadBuffer;

    // Robot Objects
    public ArrayList<RobotPart> partList = new ArrayList<>();
    public FinalLift finalLift;
    public Motion motion;
    public Sorter sorter;
    public BallLifter ballLifter;
    public Launcher launcher;
    public Intake intake;
    public Eye eye;

    // Telemetry
    static TelemetryManager telemetryM;

    @Override public void init() {
        // Setup telemetry
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Setup gamepad buffer
        gamepadBuffer = new GamepadBuffer();

        // Setup robot
        motion = new Motion(this, ignoreGamepad);
        sorter = new Sorter(this, ignoreGamepad);
        ballLifter = new BallLifter(this, ignoreGamepad);
        finalLift = new FinalLift(this, ignoreGamepad);
        launcher = new Launcher(this, ignoreGamepad);
        intake = new Intake(this);
        eye = new Eye(this);

        // Place parts into parts list
        partList.add(motion);
        partList.add(sorter);
        partList.add(ballLifter);
        partList.add(finalLift);
        partList.add(launcher);
        partList.add(intake);
        partList.add(eye);

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
        // Init gamepad
        if(!ignoreGamepad)
            gamepadBuffer.update(gamepad1, gamepad2);
    }
    @Override public void start() {
        // Reset timer and launch Threads
        opmodeTimer.resetTimer();

        // Start each part
        for( Thread part : partList)
            part.start();
    }
    @Override public void loop() {
    }
    @Override public void stop() {
        // Start each part
        for( RobotPart part : partList)
            part.safeStop();

        // Wait for threads to complete
        for( Thread part : partList) {
            try {
                part.join();
            } catch (InterruptedException ignored) {
            }
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
}
