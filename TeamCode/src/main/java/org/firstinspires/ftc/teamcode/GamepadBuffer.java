package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Test class to see if we get better performance with just a single thread watching gamepad state.
 * The other neat thing is we could listen for edges here and then trigger events instead of
 * having lots of threads polling gamepad buttons.
 */
public class GamepadBuffer {

    // Driver gamepad values
    public float g1LeftStickX;     // Strafe left/right
    public float g1LeftStickY;     // Drive Forward/Backward
    public float g1RightStickX;    // Rotate clockwise/counterclockwise
    public boolean g1Start;        // Enter final lift mode
    public boolean g1DpadUp;       // Holding start and pressing dpad up will lift robot
    public boolean g1DpadDown;     // Holding start and pressing dpad down will lower robot
    public float g1RightTrigger;   // Holding the right trigger will align with targets
    public boolean g1RightBumper;  // Holding the right bumper will keep the robot facing the target
    public float g1LeftTrigger;    // Holding the left trigger will slow the robot by the amount pressed
    public boolean g1LeftBumper;   // Holding the left bumper will slow robot to half speed

    // Tool gamepad values
    public float g2LeftStickX;     // Intake left/right
    public float g2LeftStickY;     // Intake forward/all
    public float g2RightStickX;    // Outtake left/right
    public float g2RightStickY;    // Outtake forward/all
    public float g2LeftTrigger;    // Get correct launch motor speed
    public float g2RightTrigger;   // Launches ball
    public boolean g2LeftBumper;   // Forces purple ball to launcher
    public boolean g2RightBumper;  // Forces green ball to launcher
    public boolean g2DpadLeft;     // dpad left forces left ball to launcher
    public boolean g2DpadUp;       // dpad up forces front ball to launcher
    public boolean g2DpadRight;    // dpad right forces right ball to launcher
    public boolean g2DpadDown;     // Launch all balls in expected order

    public boolean g2a;
    public boolean g2y;

    public boolean ignoreGamepad;  // Should we even use a gamepad?

    public GamepadBuffer(boolean ignoreGamepad){
        g1LeftStickX = 0;
        g1LeftStickY = 0;
        g1RightStickX = 0;
        g1Start = false;
        g1DpadUp = false;
        g1DpadDown = false;
        g1RightTrigger = 0;
        g1RightBumper = false;

        g2LeftStickX = 0;
        g2LeftStickY = 0;
        g2RightStickX = 0;
        g2RightStickY = 0;
        g2LeftTrigger = 0;
        g2RightTrigger = 0;
        g2LeftBumper = false;
        g2RightBumper = false;
        g2DpadLeft = false;
        g2DpadUp = false;
        g2DpadRight = false;
        g2DpadDown = false;

        g2a = false;
        g2y = false;

        this.ignoreGamepad = ignoreGamepad;
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        // Update controls if we're in tele-op
        if(!ignoreGamepad) {
            // Driver gamepad (only record buttons we use)
            g1LeftStickX = gamepad1.left_stick_x;
            g1LeftStickY = gamepad1.left_stick_y;
            g1RightStickX = gamepad1.right_stick_x;
            g1Start = gamepad1.start;
            g1DpadUp = gamepad1.dpad_up;
            g1DpadDown = gamepad1.dpad_down;
            g1RightTrigger = gamepad1.right_trigger;
            g1LeftTrigger = gamepad1.left_trigger;
            g1RightBumper = gamepad1.right_bumper;
            g1LeftBumper = gamepad1.left_bumper;

            // Tool gamepad (only record buttons we use)
            g2LeftStickX = gamepad2.left_stick_x;
            g2LeftStickY = gamepad2.left_stick_y;
            g2RightStickX = gamepad2.right_stick_x;
            g2RightStickY = gamepad2.right_stick_y;
            g2LeftTrigger = gamepad2.left_trigger;
            g2RightTrigger = gamepad2.right_trigger;
            g2LeftBumper = gamepad2.left_bumper;
            g2RightBumper = gamepad2.right_bumper;
            g2DpadLeft = gamepad2.dpad_left;
            g2DpadUp = gamepad2.dpad_up;
            g2DpadRight = gamepad2.dpad_right;
            g2DpadDown = gamepad2.dpad_down;

            g2a = gamepad2.a;
            g2y = gamepad2.y;
        }
    }
}
