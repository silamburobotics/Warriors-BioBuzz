package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * [ARCHIVED / DEPRECATED]
 * Superseded by team-specific OpModes:
 * - BioBuzzCentricBlueWarriors / BioBuzzCentricRedWarriors
 * - BioBuzzCentricBlueStrikers / BioBuzzCentricRedStrikers
 */
@Disabled
@Config
@TeleOp(name = "BioBuzz Robo Centric [ARCHIVED]", group = "TeleOp")
public class BioBuzzRoboCentric extends LinearOpMode {

    // --- Mecanum Drive Configuration (Gamepad 1) ---
    public static String LEFT_FRONT_NAME = "leftFront";
    public static String LEFT_BACK_NAME = "leftBack";
    public static String RIGHT_FRONT_NAME = "rightFront";
    public static String RIGHT_BACK_NAME = "rightBack";
    public static double DRIVE_POWER_SCALE = 1.0;

    // --- Intake Motor & Dual Synchronized Lift Servos Configuration (Gamepad 1) ---
    public static String INTAKE_NAME = "intake";
    public static double INTAKE_POWER = 1.0;

    public static String INTAKE_LIFT_LEFT_NAME = "intakeLiftLeft";
    public static String INTAKE_LIFT_RIGHT_NAME = "intakeLiftRight";

    // INDEPENDENT TUNABLE PARAMETERS FOR UP AND DOWN (RIGHT SERVO AUTOMATICALLY MIRRORED IN SYNC)
    public static double INTAKE_LIFT_UP_POS = 0.40;       // Tune UP height position as needed (0.0 to 1.0)
    public static double INTAKE_LIFT_DOWN_POS = 0.80;     // Tune DOWN height position as needed (0.0 to 1.0)
    public static double RIGHT_SERVO_TRIM_OFFSET = 0.0;   // Optional fine trim offset for right servo

    // --- Axon Gate & Shifter Servo Configuration (Gamepad 2) ---
    public static double OPEN_POSITION = 0.4;           // Gate Open position
    public static double CLOSED_POSITION = 0.7;        // Gate Closed position
    public static String GATE_SERVO = "gateServo";

    public static double SHIFTER_OPEN_POSITION = 0.25;     // Shifter Open position
    public static double SHIFTER_CLOSED_POSITION = 0.95;   // Shifter Closed position
    public static String SHIFTER_SERVO = "shifterServo";

    // --- Turret Motor Configuration ---
    public static String TURRET_NAME = "turret";
    public static double TURRET_TICKS_PER_REV = 537.7;     // GoBILDA 312 RPM output encoder resolution
    public static double TURRET_GEAR_RATIO = 2.2727;       // GoBILDA 175t:77t turret gear ratio (175.0 / 77.0)
    public static double TURRET_MAX_POWER = 0.35;          // Capped power limit for smooth movement (35%)
    public static double TURRET_MAX_VELOCITY_DEG_PER_SEC = 90.0; // Max speed cap in degrees/sec (90°/sec)
    public static double TURRET_MAX_ANGLE_DEG = 90.0;      // +/- 90 degrees limit from center home
    public static boolean TURRET_REVERSED = false;         // Set true if turret rotates opposite

    // --- GoBILDA 6000 RPM Flywheel Motors Configuration (Gamepad 2) ---
    public static double FLYWHEEL_OPEN_RPM = 2520.0;     // Target RPM when shifter is OPEN / Nectar mode (0.25)
    public static double FLYWHEEL_CLOSED_RPM = 2420.0;   // Target RPM when shifter is CLOSED / Pollen mode (0.95)
    public static double TICKS_PER_REV = 28.0;          // GoBILDA 6000 RPM motor encoder resolution (28 PPR)
    public static boolean MOTOR_2_REVERSED = true;      // Set true if motor 2 spins opposite to motor 1
    public static boolean FLYWHEEL_ENABLED = false;     // Toggle flywheel motors ON/OFF

    public static String MOTOR_1_NAME = "flyWheel1";
    public static String MOTOR_2_NAME = "flyWheel2";

    public enum IntakeMode {
        STOPPED,
        FORWARD,
        REVERSE
    }

    // State variables
    private boolean isIntakeLiftUp = true;               // Default UP position
    private boolean isGateOpen = false;                 // false = Closed (0.7), true = Open (0.5)
    private boolean isShifterOpen = false;              // false = Closed (0.95), true = Open (0.25)
    private double targetTurretAngle = 0.0;             // Target angle in degrees (-90.0 to +90.0)
    private double smoothedTargetRPM = 0.0;             // Slew-rate filtered target RPM to eliminate overshoot
    private IntakeMode intakeMode = IntakeMode.STOPPED;

    // Alliance & Auto-Aim State variables
    protected boolean isRedAlliance = false;             // False = Blue Alliance, True = Red Alliance
    private boolean autoAimActive = true;

    // Vision Subsystem
    private LimelightShooterSubsystem visionSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 1. Mecanum Drive Motors
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, LEFT_FRONT_NAME);
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, LEFT_BACK_NAME);
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT_NAME);
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, RIGHT_BACK_NAME);

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 2. Intake Motor & Dual Synchronized Intake Lift Servos
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo intakeLiftLeft;
        Servo intakeLiftRight;

        try {
            intakeLiftLeft = hardwareMap.get(Servo.class, INTAKE_LIFT_LEFT_NAME);
        } catch (Exception e) {
            intakeLiftLeft = null;
        }

        try {
            intakeLiftRight = hardwareMap.get(Servo.class, INTAKE_LIFT_RIGHT_NAME);
        } catch (Exception e) {
            intakeLiftRight = null;
        }

        // Set initial intake lift position to UP by default
        isIntakeLiftUp = true;
        double initLeftPos = INTAKE_LIFT_UP_POS;
        double initRightPos = Range.clip((1.0 - initLeftPos) + RIGHT_SERVO_TRIM_OFFSET, 0.0, 1.0);
        if (intakeLiftLeft != null) intakeLiftLeft.setPosition(initLeftPos);
        if (intakeLiftRight != null) intakeLiftRight.setPosition(initRightPos);

        // 3. Axon Gate & Shifter Servos
        Servo axonGateServo = hardwareMap.get(Servo.class, GATE_SERVO);
        Servo shifterServo = hardwareMap.get(Servo.class, SHIFTER_SERVO);

        isGateOpen = false;
        isShifterOpen = false;
        axonGateServo.setPosition(CLOSED_POSITION);
        shifterServo.setPosition(SHIFTER_CLOSED_POSITION);

        // 4. Shooter Motors (Master M1 has encoder, Slave M2 synced to M1 power)
        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, MOTOR_1_NAME);
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, MOTOR_2_NAME);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // 5. Turret Motor
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, TURRET_NAME);
        turret.setDirection(TURRET_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(TURRET_MAX_POWER);

        // 6. Initialize Limelight Shooter Subsystem
        visionSubsystem = new LimelightShooterSubsystem(hardwareMap);

        // Wait for Start with Alliance Color toggle
        while (!isStarted() && !isStopRequested()) {
            boolean prevAlliance = isRedAlliance;
            if (gamepad1.x || gamepad2.x) {
                isRedAlliance = false;
            } else if (gamepad1.b || gamepad2.b) {
                isRedAlliance = true;
            }
            if (prevAlliance != isRedAlliance && visionSubsystem != null) {
                visionSubsystem.resetLock();
            }

            telemetry.addData("Status", "Initialized BioBuzz Robo Centric!");
            telemetry.addData("Camera Hardware", visionSubsystem.getCameraName());
            telemetry.addData("Alliance Color", isRedAlliance ? "RED ALLIANCE [Tags 31,32,35,36]" : "BLUE ALLIANCE [Tags 39,40,43,44]");
            telemetry.addData("Intake Lift State", "DEFAULT UP (Left: %.2f | Right: %.2f)", INTAKE_LIFT_UP_POS, initRightPos);
            telemetry.addData("Gate State", "CLOSED (%.2f)", CLOSED_POSITION);
            telemetry.addData("Shifter State", "CLOSED (%.2f)", SHIFTER_CLOSED_POSITION);
            telemetry.addData("Turret State", "CENTER HOME (0°)");
            telemetry.update();
            sleep(20);
        }

        boolean lastIntakeAState = false;
        boolean lastIntakeBState = false;
        boolean lastIntakeLBState = false;
        boolean lastShifterAState = false;
        boolean lastShooterYState = false;
        boolean lastAutoAimLBState = false;
        double lastIntakePower = Double.NaN;

        while (opModeIsActive()) {

            // Update Limelight Vision Subsystem once per loop filtered strictly by active alliance tags
            visionSubsystem.update(isRedAlliance);

            // ==========================================
            // GAMEPAD 1: MECANUM DRIVE & INTAKE
            // ==========================================
            double y = -gamepad1.left_stick_y;    // Forward / Backward (Left Stick Y)
            double x = gamepad1.left_stick_x;     // Strafe Left / Right (Left Stick X)
            double rx = -gamepad1.right_stick_x;  // Turn Left / Right (Right Stick X)

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double leftFrontPower = ((y + x + rx) / denominator) * DRIVE_POWER_SCALE;
            double leftBackPower = ((y - x + rx) / denominator) * DRIVE_POWER_SCALE;
            double rightFrontPower = ((y - x - rx) / denominator) * DRIVE_POWER_SCALE;
            double rightBackPower = ((y + x - rx) / denominator) * DRIVE_POWER_SCALE;

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            // --- Intake Motor Control (Gamepad 1 - Button A: Forward, Button B: Reverse) ---
            if (gamepad1.a && !lastIntakeAState) {
                intakeMode = (intakeMode == IntakeMode.FORWARD) ? IntakeMode.STOPPED : IntakeMode.FORWARD;
            }
            lastIntakeAState = gamepad1.a;

            if (gamepad1.b && !lastIntakeBState) {
                intakeMode = (intakeMode == IntakeMode.REVERSE) ? IntakeMode.STOPPED : IntakeMode.REVERSE;
            }
            lastIntakeBState = gamepad1.b;

            double intakePower = 0.0;
            if (intakeMode == IntakeMode.FORWARD) {
                intakePower = INTAKE_POWER;
            } else if (intakeMode == IntakeMode.REVERSE) {
                intakePower = -INTAKE_POWER;
            }

            if (intakePower != lastIntakePower) {
                intake.setPower(intakePower);
                lastIntakePower = intakePower;
            }

            // --- Synchronized Dual Intake Lift Servos Control (Gamepad 1 - Left Bumper: Toggle UP / DOWN) ---
            if (gamepad1.left_bumper && !lastIntakeLBState) {
                isIntakeLiftUp = !isIntakeLiftUp;
            }
            lastIntakeLBState = gamepad1.left_bumper;

            double targetLeftLiftPos = isIntakeLiftUp ? INTAKE_LIFT_UP_POS : INTAKE_LIFT_DOWN_POS;
            double targetRightLiftPos = Range.clip((1.0 - targetLeftLiftPos) + RIGHT_SERVO_TRIM_OFFSET, 0.0, 1.0);

            if (intakeLiftLeft != null) {
                intakeLiftLeft.setPosition(targetLeftLiftPos);
            }
            if (intakeLiftRight != null) {
                intakeLiftRight.setPosition(targetRightLiftPos);
            }

            // ==========================================
            // GAMEPAD 2: GATE, SHIFTER & SHOOTER
            // ==========================================

            // --- Axon Gate Servo (G2.B = Open, G2.X = Close) ---
            if (gamepad2.b) {
                isGateOpen = true;
            } else if (gamepad2.x) {
                isGateOpen = false;
            }
            double targetGatePos = isGateOpen ? OPEN_POSITION : CLOSED_POSITION;
            axonGateServo.setPosition(targetGatePos);

            // --- Shifter Servo (Gamepad 2: Button A = Toggle Open / Closed) ---
            if (gamepad2.a && !lastShifterAState) {
                isShifterOpen = !isShifterOpen;
            }
            lastShifterAState = gamepad2.a;

            double targetShifterPos = isShifterOpen ? SHIFTER_OPEN_POSITION : SHIFTER_CLOSED_POSITION;
            shifterServo.setPosition(targetShifterPos);

            // --- Auto-Aim Toggle (Gamepad 2: Left Bumper) ---
            if (gamepad2.left_bumper && !lastAutoAimLBState) {
                autoAimActive = !autoAimActive;
                if (!autoAimActive) {
                    visionSubsystem.resetLock();
                }
            }
            lastAutoAimLBState = gamepad2.left_bumper;

            // --- Turret Auto-Aim to AprilTag when active, otherwise Center Home (0.0°) ---
            double ticksPerDegree = (TURRET_TICKS_PER_REV * TURRET_GEAR_RATIO) / 360.0;
            double currentTurretAngle = turret.getCurrentPosition() / ticksPerDegree;

            if (autoAimActive && visionSubsystem.isTargetVisible()) {
                if (Math.abs(visionSubsystem.getTxError()) > LimelightShooterSubsystem.AIM_DEADBAND_DEG) {
                    targetTurretAngle = visionSubsystem.getTurretTargetAngle(currentTurretAngle);
                }
            } else {
                targetTurretAngle = 0.0; // Center home position when target is not visible or auto-aim is OFF
            }
            targetTurretAngle = Range.clip(targetTurretAngle, -TURRET_MAX_ANGLE_DEG, TURRET_MAX_ANGLE_DEG);

            int targetTurretTicks = (int) Math.round(targetTurretAngle * ticksPerDegree);
            turret.setTargetPosition(targetTurretTicks);

            double maxTurretVelocity = TURRET_MAX_VELOCITY_DEG_PER_SEC * ticksPerDegree;
            turret.setVelocity(maxTurretVelocity);
            turret.setPower(TURRET_MAX_POWER);

            // --- Shooter Flywheel Control (Gamepad 2: Y = Toggle ON/OFF) ---
            if (gamepad2.y && !lastShooterYState) {
                FLYWHEEL_ENABLED = !FLYWHEEL_ENABLED;
            }
            lastShooterYState = gamepad2.y;

            shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
            shooter2.setDirection(MOTOR_2_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

            // Dynamic distance-based flywheel RPM calculation for Pollen (isShifterOpen = false) vs Nectar (isShifterOpen = true)
            double currentTargetRPM;
            if (autoAimActive && visionSubsystem.isTargetVisible()) {
                currentTargetRPM = visionSubsystem.getFlywheelTargetRPM(isShifterOpen);
            } else {
                currentTargetRPM = isShifterOpen ? FLYWHEEL_OPEN_RPM : FLYWHEEL_CLOSED_RPM;
            }

            // Target RPM Slew Rate Limiter to eliminate momentum overshoot
            double desiredRPM = FLYWHEEL_ENABLED ? currentTargetRPM : 0.0;
            double maxRPMChangePerLoop = 60.0; // Smooth RPM ramp rate limit (~1800 RPM/sec)
            smoothedTargetRPM += Range.clip(desiredRPM - smoothedTargetRPM, -maxRPMChangePerLoop, maxRPMChangePerLoop);

            // Master-Slave Synchronized Flywheel Control (M1 closed-loop, M2 follows M1 power)
            if (smoothedTargetRPM > 50.0) {
                double targetTPS = (smoothedTargetRPM / 60.0) * TICKS_PER_REV;
                shooter1.setVelocity(targetTPS);
                shooter2.setPower(shooter1.getPower());
            } else {
                shooter1.setVelocity(0.0);
                shooter2.setPower(0.0);
            }

            double actualTPS1 = shooter1.getVelocity();
            double actualRPM1 = Math.abs((actualTPS1 * 60.0) / TICKS_PER_REV);

            // ==========================================
            // TELEMETRY
            // ==========================================
            telemetry.addData("--- Alliance ---", isRedAlliance ? "RED ALLIANCE [Tags 31,32,35,36]" : "BLUE ALLIANCE [Tags 39,40,43,44]");
            telemetry.addData("Mecanum Drive Powers", "LF:%.2f LB:%.2f RF:%.2f RB:%.2f",
                    leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            telemetry.addData("Intake Motor (G1.A/B)", "%s (%.2f)", intakeMode.name(), intakePower);
            telemetry.addData("Intake Lift Arms (G1.LB)", isIntakeLiftUp ? "UP (L:%.2f | R:%.2f)" : "DOWN (L:%.2f | R:%.2f)",
                    targetLeftLiftPos, targetRightLiftPos);

            telemetry.addData("--- Vision & Shooter Status ---", "");
            if (visionSubsystem.isTargetVisible()) {
                boolean isAimed = visionSubsystem.isAimed();
                telemetry.addData("Limelight Target", "%s Tag #%d | tx: %.1f° | ty: %.1f° | Dist: %.1fin",
                        isAimed ? "[LOCKED ON TARGET - READY TO SHOOT!]" : "[ALIGNING...]",
                        visionSubsystem.getDetectedTagId(), visionSubsystem.getTx(), visionSubsystem.getTy(), visionSubsystem.getCalculatedDistanceInches());
            } else {
                telemetry.addData("Limelight Target", "[SEARCHING] Looking for %s tags...",
                        isRedAlliance ? "RED (31,32,35,36)" : "BLUE (39,40,43,44)");
            }

            telemetry.addData("Turret Auto-Aim (G2.LB)", autoAimActive ? "ACTIVE" : "OFF");
            telemetry.addData("Turret Angle", "Target: %.1f° | Current: %.1f° (%d ticks)",
                    targetTurretAngle, currentTurretAngle, turret.getCurrentPosition());
            telemetry.addData("Shooter Flywheel (G2.Y)", FLYWHEEL_ENABLED ? "RUNNING (Target: %.0f RPM)" : "STOPPED", currentTargetRPM);
            telemetry.addData("Shooter Motors Measured", "M1 (Master): %.1f RPM | M2 (Slave): Synced (%.0f%% Pwr)",
                    actualRPM1, shooter2.getPower() * 100.0);

            telemetry.update();
        }

        if (visionSubsystem != null) {
            visionSubsystem.stop();
        }
    }
}
