#include "NRL.h"
#include "LineActions.h"

// ============================================================
// PART 1: DEFINE HARDWARE FOR TEAM 265 (ROBOFALCONS)
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

// Arm servo (SERVO_1 / GPIO 39)
static HexaServo armServo {
    {
        .signalPin = SERVO_1,
        .startAngle = 105.0f,   // Lowered travel position (105 deg)
        .settleMs = 0
    }
};

// Gripper servo (SERVO_2 / GPIO 40)
static HexaServo gripperServo {
    {
        .signalPin = SERVO_2,
        .startAngle = 80.0f,    // Closed position (80 deg)
        .settleMs = 0
    }
};

// Third servo (SERVO_3 / GPIO 41)
static HexaServo servo3 {
    {
        .signalPin = SERVO_3,
        .startAngle = 95.0f,    // Closed position (95 deg)
        .settleMs = 0
    }
};

// TankDrive drivetrain
static TankDrive drive { leftMotor, rightMotor };

// 8-channel Arc IR Line Array on HEXA Command Hub mux pins:
// S0=GPIO18, S1=GPIO15, S2=GPIO16, SIG=GPIO17 (ADC2_CH6)
// (Note: If sensor is mounted reversed, set cfg.reversed = true)
static HexaLineArray line { hexaLineConfig() };

// Line Navigation tuning parameters for Team 265
static LineTuning team265LineTuning() {
    LineTuning t;
    t.pid.kP        = 0.50f;   // Steering strength (raise if it cuts corners, lower if it zig-zags)
    t.pid.kI        = 0.00f;   // Always 0 for line following (prevents overshoot)
    t.pid.kD        = 0.04f;   // Damping after curves / turns
    t.followSpeed   = 0.35f;   // Base forward cruise speed (0.0 to 1.0)
    t.junctionSpeed = 0.25f;   // Cautious speed while crossing junctions
    t.pivotMs       = 60;      // Roll forward ms so chassis center sits on junction before turning
    t.pivotSpeed    = 0.25f;   // Turn speed during pivot
    return t;
}

// LineActions autonomous navigation engine
static LineActions lineActions { drive, line, team265LineTuning() };


// ============================================================
// PART 2: CALIBRATED SERVO ANGLES (Team 265)
// ============================================================
constexpr float ARM_UP_ANGLE        = 35.0f;
constexpr float ARM_DOWN_ANGLE      = 105.0f;
constexpr float GRIPPER_OPEN_ANGLE  = 45.0f;
constexpr float GRIPPER_CLOSE_ANGLE = 80.0f;
constexpr float SERVO3_OPEN_ANGLE   = 135.0f;
constexpr float SERVO3_CLOSE_ANGLE  = 95.0f;


// ============================================================
// PART 3: AUTONOMOUS OPMODE IMPLEMENTATION
// ============================================================

class AutoLineNav : public NRLOpMode {

public:
    void init() override {
        // Initialize drive motors and servos
        leftMotor.begin();
        rightMotor.begin();
        armServo.begin();
        gripperServo.begin();
        servo3.begin();

        // Initialize Arc 8 line sensor and load NVS calibration
        line.begin();

        // Set safe initial servo positions
        armServo.setPosition(ARM_DOWN_ANGLE);
        gripperServo.setPosition(GRIPPER_CLOSE_ANGLE);
        servo3.setPosition(SERVO3_CLOSE_ANGLE);
    }

    void start() override {
        // Queue the autonomous line-following routine once at match start.
        // Customize this route to match your game arena layout:
        runAction(lineActions.auto_()
            .leaveStart()                           // 1. Roll out of start box until line is acquired
            .followToJunction(LineSide::ANY, 1)     // 2. Follow line to first junction and stop
            .straight()                             // 3. Cross junction straight
            .followToEnd()                          // 4. Follow line to the end marker
            .stop()                                 // 5. Safely brake motors
            .build());
    }

    void loop() override {
        // Telemetry streams at 100 Hz to the Handheld Controller display:
        telemetry.addData("Status", isActionRunning() ? "RUNNING" : "DONE");
        telemetry.addData("Calibrated", line.isCalibrated() ? "YES" : "NO! CALIBRATE");
        telemetry.addData("Line Pos", line.position());   // 0..7000 (3500 = dead center)
        telemetry.addData("Pattern", linePatternName(line.pattern()));
        telemetry.addData("Sensors", line.mask());        // 8-bit mask of sensors seeing black
        telemetry.addData("Junctions", lineActions.junctionCount());
        telemetry.addData("Fault", lineActions.faultName());
    }

    void stop() override {
        // Safety shutdown
        cancelActions();
        drive.stop();
        armServo.detach();
        gripperServo.detach();
        servo3.detach();
    }
};

// Register as an AUTO OpMode on the Controller's Autonomous menu
REGISTER_OPMODE(AutoLineNav, "AUTO LINE NAV", AUTO);
