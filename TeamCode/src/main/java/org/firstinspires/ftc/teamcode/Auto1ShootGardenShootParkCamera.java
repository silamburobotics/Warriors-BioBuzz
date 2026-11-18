package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Start 1: shoot, then run the Shoot Garden Shoot Park sequence. No AprilTag check.
 *
 * Shooter on (gate already open, intake off) -> wait -> intake on, wait (shooting) -> gate closed
 *   -> back 8 -> shift right 50 -> forward 4 -> shift left 3 -> shift right 4 -> wait 2 s -> shift left 48 (right lift stays UP)
 *   -> look for the red AprilTag (Limelight) -> if seen: open gate, wait -> shift right -> drive back -> shift right.
 */
@Config
@Autonomous(name = "1ShootGardenShootParkCamera", group = "Auto")
public class Auto1ShootGardenShootParkCamera extends LinearOpMode {

    public static double FLYWHEEL_RPM = 2900.0;       // auto shooter speed (teleop stays separate)
    public static double SPINUP_WAIT_SEC = 3.0;       // flywheel on, gate already open, intake off
    public static double SHOOT_WAIT_SEC = 3.0;        // intake on, gate open: shooting
    public static double FIRST_BACK_IN = 8.0;                  // move back after the gate closes
    public static double FIRST_SHIFT_RIGHT_IN = 50.0;          // shift right
    public static double FIRST_FORWARD_IN = 4.0;               // move forward after the shift right
    public static double SMALL_SHIFT_LEFT_IN = 3.0;            // small shift left
    public static double SMALL_SHIFT_RIGHT_IN = 4.0;           // small shift right
    public static double MID_WAIT_SEC = 2.0;                   // wait before the big shift left
    public static double BIG_SHIFT_LEFT_IN = 48.0;             // shift left
    public static double TAG_SEARCH_SEC = 1.0;                  // how long to look for the red AprilTag with the Limelight
    public static double A_WAIT_SEC = 3.0;                     // wait after opening the gate
    public static double A_BACK_IN = 80.0;                   // sequence A: drive back
    public static double A_SHIFT_RIGHT_IN = 34.0;              // sequence A: first sideways shift to the right (before driving back)
    public static double A_SECOND_SHIFT_RIGHT_IN = 18.0;       // sequence A: final sideways shift to the right

    private Limelight3A limelight;
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private double smoothedTargetRPM = 0.0;

    @Override
    public void runOpMode() {
        // Drive (Road Runner + Pinpoint)
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Intake
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, WarriorsConfig.INTAKE_NAME);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Gate starts OPEN. Shifter starts closed, same as teleop.
        Servo gate = hardwareMap.get(Servo.class, WarriorsConfig.GATE_SERVO);
        Servo shifter = hardwareMap.get(Servo.class, WarriorsConfig.SHIFTER_SERVO);
        gate.setPosition(WarriorsConfig.OPEN_POSITION);
        shifter.setPosition(WarriorsConfig.SHIFTER_CLOSED_POSITION);

        // Right intake lift servo, same as teleop: starts UP. (null if it is not in the robot configuration)
        Servo intakeLiftRight;
        try {
            intakeLiftRight = hardwareMap.get(Servo.class, WarriorsConfig.INTAKE_LIFT_RIGHT_NAME);
        } catch (Exception e) {
            intakeLiftRight = null;
        }
        if (intakeLiftRight != null) intakeLiftRight.setPosition(WarriorsConfig.INTAKE_LIFT_RIGHT_UP_POS);

        // Flywheels
        shooter1 = hardwareMap.get(DcMotorEx.class, WarriorsConfig.MOTOR_1_NAME);
        shooter2 = hardwareMap.get(DcMotorEx.class, WarriorsConfig.MOTOR_2_NAME);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setDirection(WarriorsConfig.MOTOR_1_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(WarriorsConfig.MOTOR_2_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // Limelight (optional: if it is missing, the red tag is never found and the gate stays closed)
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(LimelightShooterSubsystem.APRILTAG_PIPELINE_INDEX);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }

        telemetry.addData("Status", "Ready - RED Start 1 auto");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double flywheelRPM = FLYWHEEL_RPM;

        // Step 1: shooter on. Gate is already open and the intake is off.
        updateFlywheel(flywheelRPM);

        // Step 2: wait for the flywheel to spin up.
        holdFlywheel(flywheelRPM, SPINUP_WAIT_SEC, "Spinning up");

        // Step 3: intake on, then wait (gate is already open, so this is the shooting time).
        intake.setPower(WarriorsConfig.INTAKE_POWER);
        holdFlywheel(flywheelRPM, SHOOT_WAIT_SEC, "Shooting (gate already open)");

        // Step 4: gate closed.
        gate.setPosition(WarriorsConfig.CLOSED_POSITION);

        // Step 5: move back.
        moveAlongHeading(drive, -FIRST_BACK_IN);

        // Step 6: shift right.
        shiftSideways(drive, -FIRST_SHIFT_RIGHT_IN);

        // Step 7: move forward.
        moveAlongHeading(drive, FIRST_FORWARD_IN);

        // Step 8: shift left a little, then right a little.
        shiftSideways(drive, SMALL_SHIFT_LEFT_IN);
        shiftSideways(drive, -SMALL_SHIFT_RIGHT_IN);

        // Step 9: wait (flywheel keeps running).
        holdFlywheel(flywheelRPM, MID_WAIT_SEC, "Waiting");

        // Step 10: shift left.
        shiftSideways(drive, BIG_SHIFT_LEFT_IN);

        // Step 11: look for the red AprilTag with the Limelight.
        boolean redTagFound = searchForRedTag(flywheelRPM);

        // Step 12: if the red tag was seen, open the gate and wait. Otherwise skip this.
        if (redTagFound && opModeIsActive()) {
            gate.setPosition(WarriorsConfig.OPEN_POSITION);
            holdFlywheel(flywheelRPM, A_WAIT_SEC, "Waiting (gate open)");
        }

        // Steps 13-15: shift right, drive back, shift right again (always).
        shiftSideways(drive, -A_SHIFT_RIGHT_IN);        // shift right
        moveAlongHeading(drive, -A_BACK_IN);            // drive back
        shiftSideways(drive, -A_SECOND_SHIFT_RIGHT_IN); // shift right again

        stopEverything(drive, intake, gate);
    }

    /** Stop everything: intake, flywheel, drive and gate. */
    private void stopEverything(MecanumDrive drive, DcMotorEx intake, Servo gate) {
        smoothedTargetRPM = 0.0;
        shooter1.setPower(0.0);
        shooter2.setPower(0.0);
        intake.setPower(0.0);
        gate.setPosition(WarriorsConfig.CLOSED_POSITION);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        if (limelight != null) limelight.stop();
        telemetry.addData("Status", "Done - everything stopped");
        telemetry.update();
    }

    /** Looks for a red alliance AprilTag (IDs 30-37) for TAG_SEARCH_SEC seconds. True as soon as one is seen. */
    private boolean searchForRedTag(double flywheelRPM) {
        if (limelight == null) return false;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < TAG_SEARCH_SEC) {
            updateFlywheel(flywheelRPM);
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid() && result.getFiducialResults() != null) {
                for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
                    if (LimelightShooterSubsystem.isAllianceTag(f.getFiducialId(), true)) {
                        telemetry.addData("Red tag", "FOUND id %d", f.getFiducialId());
                        telemetry.update();
                        return true;
                    }
                }
            }
            telemetry.addData("Red tag", "searching... %.1f / %.1f s", timer.seconds(), TAG_SEARCH_SEC);
            telemetry.update();
            sleep(20);
        }
        telemetry.addData("Red tag", "NOT found");
        telemetry.update();
        return false;
    }

    /** Moves the robot along its current heading: positive = forward, negative = back. */
    private void moveAlongHeading(MecanumDrive drive, double inches) {
        if (!opModeIsActive()) return;
        Pose2d here = drive.localizer.getPose();
        double heading = here.heading.toDouble();
        runWithFlywheel(
                drive.actionBuilder(here)
                        .strafeTo(new Vector2d(
                                here.position.x + inches * Math.cos(heading),
                                here.position.y + inches * Math.sin(heading)))
                        .build());
    }

    /** Sideways move with the heading held: positive = LEFT, negative = RIGHT. Left of the heading is (-sin, cos). */
    private void shiftSideways(MecanumDrive drive, double leftInches) {
        if (!opModeIsActive()) return;
        Pose2d here = drive.localizer.getPose();
        double heading = here.heading.toDouble();
        runWithFlywheel(
                drive.actionBuilder(here)
                        .strafeTo(new Vector2d(
                                here.position.x - leftInches * Math.sin(heading),
                                here.position.y + leftInches * Math.cos(heading)))
                        .build());
    }

    /** Like Actions.runBlocking, but keeps the flywheel control loop running while the action runs. */
    private void runWithFlywheel(Action action) {
        boolean running = true;
        while (opModeIsActive() && running) {
            updateFlywheel(FLYWHEEL_RPM);
            TelemetryPacket packet = new TelemetryPacket();
            running = action.run(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    /** Runs the flywheel at targetRPM for the given time, using the same control as teleop. */
    private void holdFlywheel(double targetRPM, double seconds, String label) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            double rpm1 = updateFlywheel(targetRPM);
            telemetry.addData("Step", label);
            telemetry.addData("Time", "%.1f / %.1f s", timer.seconds(), seconds);
            telemetry.addData("Flywheel RPM", "target %.0f | measured %.0f", targetRPM, rpm1);
            telemetry.update();
            sleep(20);
        }
    }

    /** One control step, same slew limit and proportional correction as WarriorsConfig. */
    private double updateFlywheel(double targetRPM) {
        double maxRPMChangePerLoop = 60.0;
        smoothedTargetRPM += Range.clip(targetRPM - smoothedTargetRPM,
                -maxRPMChangePerLoop, maxRPMChangePerLoop);

        double actualRPM1 = (shooter1.getVelocity() * 60.0) / WarriorsConfig.TICKS_PER_REV;

        double power = 0.0;
        if (smoothedTargetRPM > 50.0) {
            double basePower = smoothedTargetRPM / 6000.0;
            double errorRPM = smoothedTargetRPM - actualRPM1;
            double kP = 0.00018;
            power = Range.clip(basePower + (errorRPM * kP), 0.0, 1.0);
        }
        shooter1.setPower(power);
        shooter2.setPower(power);
        return actualRPM1;
    }
}
