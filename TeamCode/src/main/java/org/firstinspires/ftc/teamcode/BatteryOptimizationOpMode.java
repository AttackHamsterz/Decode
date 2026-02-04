package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "Battery Optimization", group = "Robot")
public class BatteryOptimizationOpMode extends StandardSetupOpMode {

    // Configuration variables
    private static final double TARGET_VOLTAGE = 12.9;
    private static final long MAX_RUNTIME_MS = 10 * 60 * 1000; // 10 minutes
    private static final long VOLTAGE_STABILIZATION_MS = 1000;
    private static final long VOLTAGE_CHECK_INTERVAL_MS = 10 * 1000; // 10 seconds
    private static final long COMPONENT_STAGGER_MS = 500;
    private static final double LAUNCHER_RPM = 3000.0;
    private static final double WIGGLE_POWER = 0.15;
    private static final long WIGGLE_DURATION_MS = 200;
    private static final long WIGGLE_INTERVAL_MS = 2000; // Time between wiggles
    private static final int SORTER_QUARTER_TURNS = 2; // Rotate this many quarter turns each cycle
    private static final long SORTER_COMMAND_INTERVAL_MS = 1000; // Time between sorter commands

    private enum Status {
        STARTING("Starting components..."),
        DRAINING("Draining battery..."),
        CHECKING("Checking voltage..."),
        COMPLETE("Optimization complete!"),
        TIMEOUT("Timeout reached");

        private final String message;
        Status(String message) { this.message = message; }
        public String getMessage() { return message; }
    }

    private volatile Status currentStatus = Status.STARTING;
    private long lastCheckTime = 0;
    private long lastWiggleTime = 0;
    private long lastSorterCommandTime = 0;
    private volatile double currentVoltage = 0;
    private volatile boolean componentsRunning = false;
    private Thread optimizationThread;

    @Override
    public void init() {
        ignoreGamepad = true;
        super.init();

        // Read initial voltage
        currentVoltage = readVoltage();

        telemetry.addData("Status", "Ready to optimize");
        telemetry.addData("Current Voltage", String.format("%.2fV", currentVoltage));
        telemetry.addData("Target Voltage", TARGET_VOLTAGE + "V");
        telemetry.addLine("Press START to begin");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();

        // Start the threads we need
        launcher.start();
        intake.start();
        sorter.start();

        // Initialize motion follower for wiggle
        motion.follower.startTeleopDrive();

        // Start the optimization thread
        optimizationThread = new Thread(this::runOptimization);
        optimizationThread.start();
    }

    @Override
    public void loop() {
        // Update follower for wiggle movements
        motion.follower.update();

        double elapsedSeconds = opmodeTimer.getElapsedTimeSeconds();
        int minutes = (int) (elapsedSeconds / 60);
        int seconds = (int) (elapsedSeconds % 60);

        telemetry.addData("Status", currentStatus.getMessage());
        telemetry.addData("Current Voltage", String.format("%.2fV", currentVoltage));
        telemetry.addData("Target Voltage", String.format("%.2fV", TARGET_VOLTAGE));
        telemetry.addData("Elapsed Time", String.format("%d:%02d", minutes, seconds));
        telemetry.update();

        // Auto-stop when complete or timeout
        if (currentStatus == Status.COMPLETE || currentStatus == Status.TIMEOUT) {
            requestOpModeStop();
        }
    }

    @Override
    public void stop() {
        // Stop optimization thread
        if (optimizationThread != null) {
            optimizationThread.interrupt();
        }

        // Ensure all components are stopped
        stopAllComponents();

        // Stop the robot part threads
        launcher.safeStop();
        intake.safeStop();
        sorter.safeStop();

        try {
            launcher.join(1000);
        } catch (InterruptedException ignored) {}
        try {
            intake.join(1000);
        } catch (InterruptedException ignored) {}
        try {
            sorter.join(1000);
        } catch (InterruptedException ignored) {}

        // Stop motion
        motion.follower.setTeleOpDrive(0, 0, 0, true);
        motion.follower.update();
    }

    private void runOptimization() {
        try {
            // Read initial voltage
            currentVoltage = readVoltage();

            // Check if already at target
            if (currentVoltage <= TARGET_VOLTAGE) {
                currentStatus = Status.COMPLETE;
                return;
            }

            while (!Thread.currentThread().isInterrupted()) {
                // Check for timeout
                if (opmodeTimer.getElapsedTimeSeconds() * 1000 >= MAX_RUNTIME_MS) {
                    currentStatus = Status.TIMEOUT;
                    stopAllComponents();
                    return;
                }

                // Start components if not running
                if (!componentsRunning) {
                    currentStatus = Status.STARTING;
                    startAllComponents();
                    componentsRunning = true;
                    lastCheckTime = System.currentTimeMillis();
                    lastWiggleTime = System.currentTimeMillis();
                    lastSorterCommandTime = System.currentTimeMillis();
                }

                currentStatus = Status.DRAINING;

                long now = System.currentTimeMillis();

                // Perform wheel wiggle periodically
                if (now - lastWiggleTime >= WIGGLE_INTERVAL_MS) {
                    performWheelWiggle();
                    lastWiggleTime = System.currentTimeMillis();
                }

                // Keep sorter spinning by sending new rotation commands
                if (now - lastSorterCommandTime >= SORTER_COMMAND_INTERVAL_MS) {
                    sorter.rotateClockwise(SORTER_QUARTER_TURNS);
                    lastSorterCommandTime = System.currentTimeMillis();
                }

                // Check if it's time to measure voltage
                if (now - lastCheckTime >= VOLTAGE_CHECK_INTERVAL_MS) {
                    currentStatus = Status.CHECKING;

                    // Stop all components
                    stopAllComponents();
                    componentsRunning = false;

                    // Wait for voltage to stabilize
                    Thread.sleep(VOLTAGE_STABILIZATION_MS);

                    // Read voltage
                    currentVoltage = readVoltage();

                    // Check if we've reached target
                    if (currentVoltage <= TARGET_VOLTAGE) {
                        currentStatus = Status.COMPLETE;
                        return;
                    }

                    // Reset for next drain cycle
                    lastCheckTime = System.currentTimeMillis();
                }

                // Small sleep to prevent tight loop
                Thread.sleep(100);
            }
        } catch (InterruptedException e) {
            stopAllComponents();
            Thread.currentThread().interrupt();
        }
    }

    private void startAllComponents() throws InterruptedException {
        // Stagger startup to avoid current spike

        // Start intakes (all three)
        intake.leftIntakeOn();
        intake.rightIntakeOn();
        intake.frontIntakeOn();
        Thread.sleep(COMPONENT_STAGGER_MS);

        // Start sorter rotation
        sorter.rotateClockwise(SORTER_QUARTER_TURNS);
        lastSorterCommandTime = System.currentTimeMillis();
        Thread.sleep(COMPONENT_STAGGER_MS);

        // Start launcher
        launcher.setVelocityRPM(LAUNCHER_RPM);
        Thread.sleep(COMPONENT_STAGGER_MS);
    }

    private void stopAllComponents() {
        // Stop intakes
        intake.leftIntakeStop();
        intake.rightIntakeStop();
        intake.frontIntakeStop();

        // Stop launcher
        launcher.setVelocityRPM(0);

        // Stop wheel movement
        motion.follower.setTeleOpDrive(0, 0, 0, true);
        motion.follower.update();

        // Note: Sorter will coast to its current target position
        // This is acceptable for voltage checking
    }

    private void performWheelWiggle() {
        // Small rotation in place to warm up drive motors
        try {
            motion.follower.setTeleOpDrive(0, 0, WIGGLE_POWER, true);
            motion.follower.update();
            Thread.sleep(WIGGLE_DURATION_MS);

            motion.follower.setTeleOpDrive(0, 0, -WIGGLE_POWER, true);
            motion.follower.update();
            Thread.sleep(WIGGLE_DURATION_MS);

            motion.follower.setTeleOpDrive(0, 0, 0, true);
            motion.follower.update();
        } catch (InterruptedException e) {
            motion.follower.setTeleOpDrive(0, 0, 0, true);
            motion.follower.update();
            Thread.currentThread().interrupt();
        }
    }

    private double readVoltage() {
        double minVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                minVoltage = Math.min(minVoltage, voltage);
            }
        }
        return minVoltage == Double.POSITIVE_INFINITY ? 0 : minVoltage;
    }
}