#include "NRL.h"

// ============================================================
// PART 1: DEFINE THE ROBOT HARDWARE
// Physical components connected to the HEXA Command Hub.
// ============================================================

// Left drive motor
static HexaDCMotor leftMotor {
    {
        .dirPin = MOTOR_L_DIR,
        .pwmPin = MOTOR_L_PWM,
        .flipped = true
    }
};

// Right drive motor
static HexaDCMotor rightMotor {
    {
        .dirPin = MOTOR_R_DIR,
        .pwmPin = MOTOR_R_PWM
    }
};

// Arm link servo (moves arm up and down on SERVO_1 / GPIO 39)
static HexaServo armServo {
    {
        .signalPin = SERVO_1,
        .startAngle = 105.0f,   // Start at full bottom (105 deg)
        .settleMs = 0           // Active holding torque
    }
};

// Gripper claw servo (opens and closes claw on SERVO_2 / GPIO 40)
static HexaServo gripperServo {
    {
        .signalPin = SERVO_2,
        .startAngle = 80.0f,    // Initial closed position (80 deg)
        .settleMs = 0           // Active holding torque
    }
};

// Third servo (SERVO_3 / GPIO 41 pin on Command Hub)
static HexaServo servo3 {
    {
        .signalPin = SERVO_3,
        .startAngle = 95.0f,    // Initial closed position (95 deg)
        .settleMs = 0           // Active holding torque
    }
};

// TankDrive drivetrain
static TankDrive drive { leftMotor, rightMotor };


// ============================================================
// PART 2: DEFINE SERVO TARGET ANGLES & RAMPING SPEEDS
// Calibrated limits for Team 265 (ROBOFALCONS)
// ============================================================

// Arm and gripper limits (active when RT is OFF)
constexpr float ARM_UP_ANGLE        = 35.0f;   // Raised arm limit angle
constexpr float ARM_DOWN_ANGLE      = 105.0f;  // Lowered arm limit angle
constexpr float GRIPPER_OPEN_ANGLE  = 45.0f;   // Gripper open limit angle
constexpr float GRIPPER_CLOSE_ANGLE = 80.0f;   // Gripper closed limit angle

// Servo 3 limits (active when RT is ON)
constexpr float SERVO3_OPEN_ANGLE   = 135.0f;  // D-pad UP target angle
constexpr float SERVO3_CLOSE_ANGLE  = 95.0f;   // D-pad DOWN target angle

// Timing-based speed ramping parameters (deg/sec) - high responsiveness
constexpr float ARM_MIN_SPEED       = 100.0f;  // Starting speed on button tap
constexpr float ARM_MAX_SPEED       = 450.0f;  // Full servo physical transit speed on hold
constexpr float ARM_RAMP_SEC        = 0.25f;   // Quick 0.25s ramp to full speed

constexpr float GRIPPER_MIN_SPEED   = 120.0f;  // Starting speed for gripper
constexpr float GRIPPER_MAX_SPEED   = 400.0f;  // Fast grip actuation speed
constexpr float GRIPPER_RAMP_SEC    = 0.20f;   // Quick 0.20s ramp to full speed

constexpr float SERVO3_MIN_SPEED    = 100.0f;  // Starting speed for servo 3
constexpr float SERVO3_MAX_SPEED    = 450.0f;  // Fast transit speed for servo 3
constexpr float SERVO3_RAMP_SEC     = 0.25f;   // Quick 0.25s ramp to full speed


// ============================================================
// PART 3: OPMODE IMPLEMENTATION
// Driver-controlled TeleOp OpMode for Team ROBOFALCONS (265).
// ============================================================

class fullcode : public NRLOpMode {

private:
    // Tracks current angles for servos
    float currentArmAngle     = ARM_DOWN_ANGLE;
    float currentGripperAngle = GRIPPER_CLOSE_ANGLE;
    float currentServo3Angle  = SERVO3_CLOSE_ANGLE;

    // Loop delta-time tracking
    uint32_t lastLoopMs = 0;

    // Button hold start timestamps (0 = not pressed)
    uint32_t armUpHoldStartMs = 0;
    uint32_t armDownHoldStartMs = 0;
    uint32_t gripperOpenHoldStartMs = 0;
    uint32_t gripperCloseHoldStartMs = 0;
    uint32_t servo3UpHoldStartMs = 0;
    uint32_t servo3DownHoldStartMs = 0;

    // Helper: moves currentAngle towards targetAngle with hold-duration acceleration
    void updateServoWithHoldRamp(HexaServo& servo, float& currentAngle,
                                 float targetAngle, uint32_t& holdStartMs,
                                 uint32_t nowMs, float dt,
                                 float minSpeed, float maxSpeed, float rampSec) {
        if (holdStartMs == 0) {
            holdStartMs = nowMs;
        }
        float holdTimeSec = (float)(nowMs - holdStartMs) / 1000.0f;
        float progress = holdTimeSec / rampSec;
        if (progress > 1.0f) progress = 1.0f;

        // Speed increases based on how long the button is held
        float speed = minSpeed + (maxSpeed - minSpeed) * progress;
        float step = speed * dt;

        // Move towards targetAngle without overshooting
        if (currentAngle < targetAngle) {
            currentAngle += step;
            if (currentAngle > targetAngle) currentAngle = targetAngle;
        } else if (currentAngle > targetAngle) {
            currentAngle -= step;
            if (currentAngle < targetAngle) currentAngle = targetAngle;
        }

        servo.setPosition(currentAngle);
    }

public:
    void init() override {
        leftMotor.begin();
        rightMotor.begin();
        armServo.begin();
        gripperServo.begin();
        servo3.begin();

        // Initialize positions to calibrated defaults
        currentArmAngle = ARM_DOWN_ANGLE;
        armServo.setPosition(currentArmAngle);

        currentGripperAngle = GRIPPER_CLOSE_ANGLE;
        gripperServo.setPosition(currentGripperAngle);

        currentServo3Angle = SERVO3_CLOSE_ANGLE;
        servo3.setPosition(currentServo3Angle);

        lastLoopMs = millis();
        armUpHoldStartMs = 0;
        armDownHoldStartMs = 0;
        gripperOpenHoldStartMs = 0;
        gripperCloseHoldStartMs = 0;
        servo3UpHoldStartMs = 0;
        servo3DownHoldStartMs = 0;
    }

    void loop() override {
        // Calculate delta time (dt) in seconds
        uint32_t nowMs = millis();
        float dt = (lastLoopMs == 0) ? 0.01f : (float)(nowMs - lastLoopMs) / 1000.0f;
        if (dt > 0.05f) dt = 0.01f;  // Clamp to prevent jump if stalled
        lastLoopMs = nowMs;

        // Drive controls: Left stick Y = forward/back, Right stick X = steering
        drive.drive(gamepad1.leftY(), gamepad1.rightX());

        // ----------------------------------------------------
        // SERVO SELECTION VIA RT TOGGLE SWITCH (GP41 on Controller)
        // ----------------------------------------------------
        bool rtMode = gamepad1.pressed(BTN_RT);

        if (rtMode) {
            // ================================================
            // RT IS ON: SERVO 3 CONTROL ONLY (SERVO_3 / GPIO 41)
            // ================================================
            armUpHoldStartMs = 0;
            armDownHoldStartMs = 0;
            gripperOpenHoldStartMs = 0;
            gripperCloseHoldStartMs = 0;

            bool upPressed = gamepad1.pressed(BTN_DPAD_UP);
            bool downPressed = gamepad1.pressed(BTN_DPAD_DOWN);

            if (upPressed && !downPressed) {
                servo3DownHoldStartMs = 0;
                updateServoWithHoldRamp(servo3, currentServo3Angle, SERVO3_OPEN_ANGLE,
                                        servo3UpHoldStartMs, nowMs, dt,
                                        SERVO3_MIN_SPEED, SERVO3_MAX_SPEED, SERVO3_RAMP_SEC);
            } else if (downPressed && !upPressed) {
                servo3UpHoldStartMs = 0;
                updateServoWithHoldRamp(servo3, currentServo3Angle, SERVO3_CLOSE_ANGLE,
                                        servo3DownHoldStartMs, nowMs, dt,
                                        SERVO3_MIN_SPEED, SERVO3_MAX_SPEED, SERVO3_RAMP_SEC);
            } else {
                servo3UpHoldStartMs = 0;
                servo3DownHoldStartMs = 0;
            }

            telemetry.addData("Mode", "SERVO 3");
            telemetry.addData("S3 Deg", currentServo3Angle);
        } else {
            // ================================================
            // RT IS OFF: SERVO 1 (ARM) & SERVO 2 (GRIPPER)
            // ================================================
            servo3UpHoldStartMs = 0;
            servo3DownHoldStartMs = 0;

            // Optional direct shortcut: BTN_Y moves arm to MID (70 deg)
            if (gamepad1.justPressed(BTN_Y)) {
                currentArmAngle = 70.0f;
                armServo.setPosition(currentArmAngle);
            }

            // Arm link control (D-Pad UP to raise, D-Pad DOWN to lower)
            bool armUpPressed = gamepad1.pressed(BTN_DPAD_UP);
            bool armDownPressed = gamepad1.pressed(BTN_DPAD_DOWN);

            if (armUpPressed && !armDownPressed) {
                armDownHoldStartMs = 0;
                updateServoWithHoldRamp(armServo, currentArmAngle, ARM_UP_ANGLE,
                                        armUpHoldStartMs, nowMs, dt,
                                        ARM_MIN_SPEED, ARM_MAX_SPEED, ARM_RAMP_SEC);
            } else if (armDownPressed && !armUpPressed) {
                armUpHoldStartMs = 0;
                updateServoWithHoldRamp(armServo, currentArmAngle, ARM_DOWN_ANGLE,
                                        armDownHoldStartMs, nowMs, dt,
                                        ARM_MIN_SPEED, ARM_MAX_SPEED, ARM_RAMP_SEC);
            } else {
                armUpHoldStartMs = 0;
                armDownHoldStartMs = 0;
            }

            // Gripper claw control (D-Pad LEFT to open, D-Pad RIGHT to close)
            bool gripperOpenPressed = gamepad1.pressed(BTN_DPAD_LEFT);
            bool gripperClosePressed = gamepad1.pressed(BTN_DPAD_RIGHT);

            if (gripperOpenPressed && !gripperClosePressed) {
                gripperCloseHoldStartMs = 0;
                updateServoWithHoldRamp(gripperServo, currentGripperAngle, GRIPPER_OPEN_ANGLE,
                                        gripperOpenHoldStartMs, nowMs, dt,
                                        GRIPPER_MIN_SPEED, GRIPPER_MAX_SPEED, GRIPPER_RAMP_SEC);
            } else if (gripperClosePressed && !gripperOpenPressed) {
                gripperOpenHoldStartMs = 0;
                updateServoWithHoldRamp(gripperServo, currentGripperAngle, GRIPPER_CLOSE_ANGLE,
                                        gripperCloseHoldStartMs, nowMs, dt,
                                        GRIPPER_MIN_SPEED, GRIPPER_MAX_SPEED, GRIPPER_RAMP_SEC);
            } else {
                gripperOpenHoldStartMs = 0;
                gripperCloseHoldStartMs = 0;
            }

            telemetry.addData("Mode", "ARM/GRIP");
            telemetry.addData("Arm", currentArmAngle);
            telemetry.addData("Gripper", currentGripperAngle);
        }
    }

    void stop() override {
        leftMotor.stop();
        rightMotor.stop();
        armServo.detach();
        gripperServo.detach();
        servo3.detach();
    }
};

// ============================================================
// PART 4: REGISTER THE OPMODE
// Displays on the Handheld Controller menu as "RUNBOT"
// ============================================================
REGISTER_OPMODE(fullcode, "RUNBOT", TELEOP);
