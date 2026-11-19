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
        .startAngle = 50.0f,    // Initial closed position (50 deg)
        .settleMs = 0           // Active holding torque
    }
};

// TankDrive drivetrain
static TankDrive drive { leftMotor, rightMotor };


// ============================================================
// PART 2: DEFINE 3-STOP ARM POSITIONS & SERVO LIMITS (Team 265)
// ============================================================

// 3 Arm Stop Angles: Full Top (35 deg), Middle (70 deg), Full Bottom (105 deg)
constexpr float ARM_TOP_ANGLE       = 35.0f;   // Full top limit angle
constexpr float ARM_MID_ANGLE       = 70.0f;   // Middle stop angle
constexpr float ARM_BOTTOM_ANGLE    = 105.0f;  // Full bottom limit angle

// Gripper limits for Team 265
constexpr float GRIPPER_OPEN_ANGLE  = 45.0f;   // Gripper open angle
constexpr float GRIPPER_CLOSE_ANGLE = 80.0f;   // Gripper closed angle

// Servo 3 limits (active when RT is ON)
constexpr float SERVO3_OPEN_ANGLE   = 120.0f;  // D-pad UP target angle
constexpr float SERVO3_CLOSE_ANGLE  = 50.0f;   // D-pad DOWN target angle


// ============================================================
// PART 3: OPMODE IMPLEMENTATION
// Driver-controlled TeleOp OpMode for Team ROBOFALCONS (265).
// ============================================================

class fullcode : public NRLOpMode {

private:
    // Tracks current arm stop: 0 = BOTTOM (105°), 1 = MID (70°), 2 = TOP (35°)
    int currentArmStop = 0;
    const float armStops[3] = { ARM_BOTTOM_ANGLE, ARM_MID_ANGLE, ARM_TOP_ANGLE };

    // Track state of gripper and servo 3
    float currentGripperAngle = GRIPPER_CLOSE_ANGLE;
    float currentServo3Angle  = SERVO3_CLOSE_ANGLE;

public:
    void init() override {
        leftMotor.begin();
        rightMotor.begin();
        armServo.begin();
        gripperServo.begin();
        servo3.begin();

        // Initialize positions to safe defaults
        currentArmStop = 0; // Bottom position (105°)
        armServo.setPosition(armStops[currentArmStop]);

        currentGripperAngle = GRIPPER_CLOSE_ANGLE;
        gripperServo.setPosition(currentGripperAngle);

        currentServo3Angle = SERVO3_CLOSE_ANGLE;
        servo3.setPosition(currentServo3Angle);
    }

    void loop() override {
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
            if (gamepad1.justPressed(BTN_DPAD_UP)) {
                currentServo3Angle = SERVO3_OPEN_ANGLE;
                servo3.setPosition(currentServo3Angle);
            }
            if (gamepad1.justPressed(BTN_DPAD_DOWN)) {
                currentServo3Angle = SERVO3_CLOSE_ANGLE;
                servo3.setPosition(currentServo3Angle);
            }

            telemetry.addData("Mode", "SERVO 3");
            telemetry.addData("S3 Deg", currentServo3Angle);
        } else {
            // ================================================
            // RT IS OFF: SERVO 1 (ARM) & SERVO 2 (GRIPPER)
            // ================================================

            // --- ARM 3-STOP STEPPING ---
            // D-Pad UP: Step UP to next stop (Bottom -> Mid -> Top)
            if (gamepad1.justPressed(BTN_DPAD_UP)) {
                if (currentArmStop < 2) {
                    currentArmStop++;
                    armServo.setPosition(armStops[currentArmStop]);
                }
            }

            // D-Pad DOWN: Step DOWN to next stop (Top -> Mid -> Bottom)
            if (gamepad1.justPressed(BTN_DPAD_DOWN)) {
                if (currentArmStop > 0) {
                    currentArmStop--;
                    armServo.setPosition(armStops[currentArmStop]);
                }
            }

            // Optional direct shortcut: BTN_Y immediately moves to MID
            if (gamepad1.justPressed(BTN_Y)) {
                currentArmStop = 1;
                armServo.setPosition(armStops[currentArmStop]);
            }

            // --- GRIPPER CLAW CONTROL ---
            // D-Pad LEFT to open (45°), D-Pad RIGHT to close (80°)
            if (gamepad1.justPressed(BTN_DPAD_LEFT)) {
                currentGripperAngle = GRIPPER_OPEN_ANGLE;
                gripperServo.setPosition(currentGripperAngle);
            }
            if (gamepad1.justPressed(BTN_DPAD_RIGHT)) {
                currentGripperAngle = GRIPPER_CLOSE_ANGLE;
                gripperServo.setPosition(currentGripperAngle);
            }

            telemetry.addData("Mode", "ARM/GRIP");
            telemetry.addData("Arm Stop", currentArmStop == 0 ? "BOTTOM" : (currentArmStop == 1 ? "MID" : "TOP"));
            telemetry.addData("Arm Deg", armStops[currentArmStop]);
            telemetry.addData("Gripper", currentGripperAngle == GRIPPER_OPEN_ANGLE ? "OPEN" : "CLOSED");
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
