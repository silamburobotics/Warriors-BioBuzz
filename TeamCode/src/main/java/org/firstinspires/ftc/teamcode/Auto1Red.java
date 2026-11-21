package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * RED alliance, Start 1: shoot, move, then run the path. NO camera and NO AprilTag check (use when the camera is not working).
 *
 * Opening: flywheel on (gate open) -> wait 2.5 -> intake on, wait 1 -> gate closed -> back 10 -> shift right 50
 *   -> lift DOWN -> forward 8 -> wait 1 -> lift UP -> stop intake -> shift left 48 -> start intake.
 * Then: gate open, wait 1 -> shift right 34 -> gate closed -> back 30 -> turn cw 90 -> lift DOWN -> forward 12
 *   -> wait 2 -> back 12 -> lift UP -> shift right 50 -> forward 16.
 * Intake and flywheel run to the end. Flywheel RPM comes from the battery voltage (2860 if unreadable).
 */
@Config
@Autonomous(name = "1Red", group = "Auto")
public class Auto1Red extends LinearOpMode {

    private double flywheelTargetRPM = WarriorsConfig.MID_BATTERY_RPM; // set from the battery at init


    // Opening
    public static double INITIAL_BACK_IN = 3.0;                // move back during spin-up before initial shooting
    public static double SPINUP_WAIT_SEC = 2.5;
    public static double SHOOT_WAIT_SEC = 1.0;
    public static double OPEN_BACK_IN = 12.0;
    public static double OPEN_SHIFT_RIGHT_IN = 65.5;
    public static double OPEN_FORWARD_IN = 13.5;
    public static double OPEN_WAIT_SEC = 1.0;
    public static double OPEN_SHIFT_LEFT_IN = 66.0;

    // Path (turn sign: + = anticlockwise, - = clockwise)
    public static double F_GATE_WAIT_SEC = 1.0;
    public static double F_SHIFT_RIGHT_1_IN = 40.0;
    public static double F_BACK_1_IN = 30.0;
    public static double F_TURN_1_DEG = 90.0;         // anticlockwise 90
    public static double F_FORWARD_1_IN = 17.0;
    public static double F_WAIT_1_SEC = 2.0;
    public static double F_BACK_2_IN = 12.0;
    public static double F_SHIFT_RIGHT_2_IN = 62.0;
    public static double F_FORWARD_2_IN = 20.0;


    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private double smoothedTargetRPM = 0.0;

    @Override
    public void runOpMode() {
        // Flywheel speed from battery voltage (read once at init; 2860 if unreadable)
        flywheelTargetRPM = WarriorsConfig.getBatteryAdjustedRPM(hardwareMap);

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

        // Dual Synchronized Intake Lift Servos (Left & Right), same as teleop: start UP.
        Servo intakeLiftLeft, intakeLiftRight;
        try {
            intakeLiftLeft = hardwareMap.get(Servo.class, WarriorsConfig.INTAKE_LIFT_LEFT_NAME);
        } catch (Exception e) {
            intakeLiftLeft = null;
        }
        try {
            intakeLiftRight = hardwareMap.get(Servo.class, WarriorsConfig.INTAKE_LIFT_RIGHT_NAME);
            if (intakeLiftRight != null) {
                intakeLiftRight.setDirection(WarriorsConfig.INTAKE_LIFT_RIGHT_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            }
        } catch (Exception e) {
            intakeLiftRight = null;
        }
        if (intakeLiftLeft != null) intakeLiftLeft.setPosition(WarriorsConfig.INTAKE_LIFT_UP_POS);
        if (intakeLiftRight != null) intakeLiftRight.setPosition(WarriorsConfig.INTAKE_LIFT_UP_POS);

        // Flywheels
        shooter1 = hardwareMap.get(DcMotorEx.class, WarriorsConfig.MOTOR_1_NAME);
        shooter2 = hardwareMap.get(DcMotorEx.class, WarriorsConfig.MOTOR_2_NAME);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(WarriorsConfig.MOTOR_2_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Ready - RED Start 1 shoot garden shoot flower shoot park");
        telemetry.addData("Flywheel RPM (battery)", "%.0f", flywheelTargetRPM);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double rpm = flywheelTargetRPM;

        // ---------- Opening ----------
        updateFlywheel(rpm);                                                  // 1. flywheel on, gate already open
        moveAlongHeading(drive, -INITIAL_BACK_IN);                            // 2. move 1.0" back WHILE flywheel spins up
        holdFlywheel(rpm, SPINUP_WAIT_SEC, "Spinning up");                    // 3. wait for remaining spin-up
        intake.setPower(WarriorsConfig.INTAKE_POWER);                         // 3. intake on
        holdFlywheel(rpm, SHOOT_WAIT_SEC, "Shooting (gate already open)");    //    wait while shooting
        gate.setPosition(WarriorsConfig.CLOSED_POSITION);                     // 4. gate closed
        moveAlongHeading(drive, -OPEN_BACK_IN);                               // 5. back
        shiftSideways(drive, -OPEN_SHIFT_RIGHT_IN);                           // 6. shift right
        liftDown(intakeLiftLeft, intakeLiftRight);                             // 7. lift DOWN
        moveAlongHeading(drive, OPEN_FORWARD_IN);                             // 8. forward
        holdFlywheel(rpm, OPEN_WAIT_SEC, "Waiting");                          // 9. wait
        liftUp(intakeLiftLeft, intakeLiftRight);                               // 10. lift UP
        intake.setPower(0.0);                                                 //     stop intake
        shiftSideways(drive, OPEN_SHIFT_LEFT_IN);                             // 11. shift left
        if (opModeIsActive()) intake.setPower(WarriorsConfig.INTAKE_POWER);  // 12. start intake

        // ---------- Path ----------
        gate.setPosition(WarriorsConfig.OPEN_POSITION);
        holdFlywheel(rpm, F_GATE_WAIT_SEC, "Gate open");
        shiftSideways(drive, -F_SHIFT_RIGHT_1_IN);
        gate.setPosition(WarriorsConfig.CLOSED_POSITION);
        moveAlongHeading(drive, -F_BACK_1_IN);
        turnDegrees(drive, F_TURN_1_DEG);
        liftDown(intakeLiftLeft, intakeLiftRight);
        moveAlongHeading(drive, F_FORWARD_1_IN);
        holdFlywheel(rpm, F_WAIT_1_SEC, "Waiting");
        moveAlongHeading(drive, -F_BACK_2_IN);
        liftUp(intakeLiftLeft, intakeLiftRight);
        shiftSideways(drive, -F_SHIFT_RIGHT_2_IN);
        moveAlongHeading(drive, F_FORWARD_2_IN);

        // Intake and flywheel keep running until the period ends.
        while (opModeIsActive()) {
            double rpm1 = updateFlywheel(rpm);
            telemetry.addData("Status", "Done - intake and flywheel running");
            telemetry.addData("Flywheel RPM", "target %.0f | measured %.0f", rpm, rpm1);
            telemetry.update();
            sleep(20);
        }
    }

    private void liftDown(Servo liftLeft, Servo liftRight) {
        if (liftLeft != null && opModeIsActive()) liftLeft.setPosition(WarriorsConfig.INTAKE_LIFT_DOWN_POS);
        if (liftRight != null && opModeIsActive()) liftRight.setPosition(WarriorsConfig.INTAKE_LIFT_DOWN_POS);
    }

    private void liftUp(Servo liftLeft, Servo liftRight) {
        if (liftLeft != null && opModeIsActive()) liftLeft.setPosition(WarriorsConfig.INTAKE_LIFT_UP_POS);
        if (liftRight != null && opModeIsActive()) liftRight.setPosition(WarriorsConfig.INTAKE_LIFT_UP_POS);
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

    /** Turn in place: positive = anticlockwise, negative = clockwise. */
    private void turnDegrees(MecanumDrive drive, double degrees) {
        if (!opModeIsActive()) return;
        runWithFlywheel(
                drive.actionBuilder(drive.localizer.getPose())
                        .turn(Math.toRadians(degrees))
                        .build());
    }

    /** Like Actions.runBlocking, but keeps the flywheel control loop running while the action runs. */
    private void runWithFlywheel(Action action) {
        boolean running = true;
        while (opModeIsActive() && running) {
            updateFlywheel(flywheelTargetRPM);
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
