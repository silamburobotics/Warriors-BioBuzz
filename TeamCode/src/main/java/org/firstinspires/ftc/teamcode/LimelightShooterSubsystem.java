package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/**
 * Limelight 3A vision subsystem.
 * Provides AprilTag tracking with alliance filtering and lock-on, distance to the tag,
 * turret auto-aim angles and distance-based flywheel RPM.
 */
@Config
public class LimelightShooterSubsystem {

    // Camera
    public static int APRILTAG_PIPELINE_INDEX = 0;
    public static int DASHBOARD_STREAM_FPS = 15;
    public static long MAX_RESULT_STALENESS_MS = 100;  // ignore Limelight results older than this
    public static double LOST_TARGET_HOLD_MS = 300.0;  // keep the last target this long after it disappears

    // Distance
    public static boolean USE_3D_DISTANCE = true;      // true = Limelight 3D tag pose (straight-line range); false = mount-angle trig
    public static double LIMELIGHT_LENS_HEIGHT_INCHES = 12.0;   // only used by the trig method / fallback
    public static double TARGET_CENTER_HEIGHT_INCHES = 36.0;    // only used by the trig method / fallback
    public static double LIMELIGHT_MOUNT_ANGLE_DEG = 45.0;      // only used by the trig method / fallback
    public static double MIN_DISTANCE_IN = 12.0;
    public static double MAX_DISTANCE_IN = 144.0;

    // Turret auto-aim
    public static double TURRET_AUTO_AIM_GAIN = 0.35;      // Damped gain to avoid overshoot/oscillation
    public static double TURRET_AIM_OFFSET_DEG = 0.0;      // Horizontal offset from tag center to the target basket
    public static double AIM_DEADBAND_DEG = 1.5;           // No correction inside this error
    public static double TURRET_MIN_ANGLE_DEG = -90.0;
    public static double TURRET_MAX_ANGLE_DEG = 90.0;

    // Ball-type flywheel RPM curves (Pollen = smaller ball, Nectar = bigger ball)
    public static double POLLEN_BASE_RPM = 2150.0;
    public static double POLLEN_RPM_SLOPE = 6.0;           // RPM per inch of distance
    public static double NECTAR_BASE_RPM = 2320.0;
    public static double NECTAR_RPM_SLOPE = 6.0;           // RPM per inch of distance
    public static double FLYWHEEL_MIN_RPM = 2000.0;
    public static double FLYWHEEL_MAX_RPM = 4500.0;

    // Alliance tag ID ranges
    public static int RED_TAG_MIN = 30, RED_TAG_MAX = 37;
    public static int BLUE_TAG_MIN = 38, BLUE_TAG_MAX = 45;

    private Limelight3A limelight;
    private boolean cameraActive = true;

    // Target state
    private boolean targetVisible = false;   // true while a target is seen OR being held after a brief loss
    private boolean freshMeasurement = false; // true only when this update() read a new frame with the target in it
    private double tx = 0.0;
    private double ty = 0.0;
    private double calculatedDistanceInches = 0.0;
    private int detectedTagId = -1;
    private String tagsSeenSummary = "None";

    // Lock-on state
    private int lockedTagId = -1;
    private boolean targetLocked = false;
    private final ElapsedTime sinceTargetSeen = new ElapsedTime();
    private long lastFrameTimestampNanos = -1;
    private double lastTurretTarget = Double.NaN;

    public LimelightShooterSubsystem(HardwareMap hardwareMap) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception ignored) {
            List<Limelight3A> limelights = hardwareMap.getAll(Limelight3A.class);
            limelight = (limelights != null && !limelights.isEmpty()) ? limelights.get(0) : null;
        }

        if (limelight != null) {
            try {
                limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
                limelight.start();
                FtcDashboard.getInstance().startCameraStream(limelight, DASHBOARD_STREAM_FPS);
            } catch (Exception ignored) {
                limelight = null;
            }
        }
    }

    /** True if the tag ID belongs to the given alliance (red 30-37, blue 38-45 by default). */
    public static boolean isAllianceTag(int tagId, boolean isRedAlliance) {
        return isRedAlliance
                ? tagId >= RED_TAG_MIN && tagId <= RED_TAG_MAX
                : tagId >= BLUE_TAG_MIN && tagId <= BLUE_TAG_MAX;
    }

    /** Clears the lock so a new AprilTag can be acquired. */
    public void resetLock() {
        lockedTagId = -1;
        targetLocked = false;
        clearTarget();
    }

    /**
     * Reads the newest Limelight frame, filtered to the alliance's tags.
     * Locks onto the tag nearest the crosshair and keeps it while it stays in view.
     * Call once per loop.
     */
    public void update(boolean isRedAlliance) {
        freshMeasurement = false;

        LLResult result = (limelight != null && cameraActive) ? limelight.getLatestResult() : null;
        boolean usable = result != null && result.isValid() && result.getStaleness() <= MAX_RESULT_STALENESS_MS;

        if (usable) {
            long frameTime = result.getControlHubTimeStampNanos();
            if (frameTime == lastFrameTimestampNanos) {
                return; // same frame as last loop: nothing new, keep the current state
            }
            lastFrameTimestampNanos = frameTime;

            if (processFiducials(result.getFiducialResults(), isRedAlliance)) {
                return; // target found in this frame
            }
        } else {
            tagsSeenSummary = "None";
        }

        // Target not in this frame: hold the last target briefly, then drop it.
        if (targetLocked && sinceTargetSeen.milliseconds() < LOST_TARGET_HOLD_MS) {
            targetVisible = true; // keeps distance/RPM steady; the turret holds still (see getTurretTargetAngle)
        } else {
            resetLock();
        }
    }

    /** Default update (Blue alliance). */
    public void update() {
        update(false);
    }

    /** Picks the target from this frame's tags. Returns true if an alliance tag was found. */
    private boolean processFiducials(List<LLResultTypes.FiducialResult> fiducials, boolean isRedAlliance) {
        if (fiducials == null || fiducials.isEmpty()) {
            tagsSeenSummary = "None";
            return false;
        }

        StringBuilder seen = new StringBuilder();
        LLResultTypes.FiducialResult locked = null;
        LLResultTypes.FiducialResult nearest = null;
        double bestAbsTx = Double.MAX_VALUE;

        for (LLResultTypes.FiducialResult f : fiducials) {
            int id = f.getFiducialId();
            if (seen.length() > 0) seen.append(", ");
            seen.append('#').append(id);

            if (!isAllianceTag(id, isRedAlliance)) continue;
            if (id == lockedTagId) locked = f;
            double absTx = Math.abs(f.getTargetXDegrees());
            if (absTx < bestAbsTx) {
                bestAbsTx = absTx;
                nearest = f;
            }
        }
        tagsSeenSummary = seen.toString();

        LLResultTypes.FiducialResult match = (locked != null) ? locked : nearest;
        if (match == null) return false;

        targetVisible = true;
        targetLocked = true;
        freshMeasurement = true;
        lockedTagId = match.getFiducialId();
        detectedTagId = lockedTagId;
        tx = match.getTargetXDegrees();
        ty = match.getTargetYDegrees();
        sinceTargetSeen.reset();

        double distance = USE_3D_DISTANCE ? distanceFromPose(match) : Double.NaN;
        if (Double.isNaN(distance)) distance = distanceFromTrig(ty);
        if (!Double.isNaN(distance)) {
            calculatedDistanceInches = Range.clip(distance, MIN_DISTANCE_IN, MAX_DISTANCE_IN);
        }
        return true;
    }

    /** Straight-line camera-to-tag range from the Limelight 3D pose, or NaN if the pipeline gives no pose. */
    private static double distanceFromPose(LLResultTypes.FiducialResult f) {
        try {
            Pose3D pose = f.getTargetPoseCameraSpace();
            if (pose == null || pose.getPosition() == null) return Double.NaN;
            Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
            double range = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            return range > 0.0 ? range : Double.NaN; // all zeros = 3D not enabled in the pipeline
        } catch (Exception e) {
            return Double.NaN;
        }
    }

    /** Floor distance from the mount height and angle, or NaN if the geometry is invalid. */
    private static double distanceFromTrig(double tyDeg) {
        double tan = Math.tan(Math.toRadians(LIMELIGHT_MOUNT_ANGLE_DEG + tyDeg));
        if (tan <= 0) return Double.NaN;
        return (TARGET_CENTER_HEIGHT_INCHES - LIMELIGHT_LENS_HEIGHT_INCHES) / tan;
    }

    private void clearTarget() {
        targetVisible = false;
        freshMeasurement = false;
        detectedTagId = -1;
        tx = 0.0;
        ty = 0.0;
        lastTurretTarget = Double.NaN;
    }

    public String getCameraName() {
        return limelight != null ? "Limelight 3A" : "NONE (No Limelight Detected)";
    }

    public boolean isTargetVisible() {
        return targetVisible;
    }

    public String getTagsSeenSummary() {
        return tagsSeenSummary;
    }

    public boolean isAimed() {
        return targetVisible && Math.abs(getTxError()) <= AIM_DEADBAND_DEG;
    }

    public double getTx() {
        return tx;
    }

    public double getTxError() {
        return tx - TURRET_AIM_OFFSET_DEG;
    }

    public double getTy() {
        return ty;
    }

    public double getCalculatedDistanceInches() {
        return calculatedDistanceInches;
    }

    public int getDetectedTagId() {
        return detectedTagId;
    }

    /**
     * Turret angle that lines up with the target, within [TURRET_MIN_ANGLE_DEG, TURRET_MAX_ANGLE_DEG].
     * A correction is applied only for a new camera frame. Between frames, or while the target
     * is briefly lost, the previous target angle is returned so old data is never applied twice.
     */
    public double getTurretTargetAngle(double currentTurretAngleDeg) {
        if (!targetVisible) return currentTurretAngleDeg;
        if (!freshMeasurement) {
            return Double.isNaN(lastTurretTarget) ? currentTurretAngleDeg : lastTurretTarget;
        }
        double errorTx = getTxError();
        if (Math.abs(errorTx) <= AIM_DEADBAND_DEG) {
            lastTurretTarget = currentTurretAngleDeg;
        } else {
            lastTurretTarget = Range.clip(currentTurretAngleDeg - errorTx * TURRET_AUTO_AIM_GAIN,
                    TURRET_MIN_ANGLE_DEG, TURRET_MAX_ANGLE_DEG);
        }
        freshMeasurement = false; // this frame's correction has been used
        return lastTurretTarget;
    }

    /**
     * Flywheel RPM from distance and ball type, starting from customBaseRPM.
     * isNectarMode = false: Pollen (smaller ball); true: Nectar (bigger ball).
     */
    public double getFlywheelTargetRPM(boolean isNectarMode, double customBaseRPM) {
        if (!targetVisible || calculatedDistanceInches <= 0) {
            return customBaseRPM;
        }
        double slope = isNectarMode ? NECTAR_RPM_SLOPE : POLLEN_RPM_SLOPE;
        return Range.clip(customBaseRPM + slope * calculatedDistanceInches, FLYWHEEL_MIN_RPM, FLYWHEEL_MAX_RPM);
    }

    public double getFlywheelTargetRPM(boolean isNectarMode) {
        return getFlywheelTargetRPM(isNectarMode, isNectarMode ? NECTAR_BASE_RPM : POLLEN_BASE_RPM);
    }

    public boolean isCameraActive() {
        return cameraActive;
    }

    /** Pauses / resumes the Limelight and the Dashboard stream to save CPU when auto-aim is off. No target is reported while paused. */
    public void setCameraActive(boolean active) {
        if (active == cameraActive) return;
        cameraActive = active;
        if (limelight == null) return;
        try {
            if (active) {
                limelight.start(); // resumes polling after pause()
                FtcDashboard.getInstance().startCameraStream(limelight, DASHBOARD_STREAM_FPS);
            } else {
                FtcDashboard.getInstance().stopCameraStream();
                limelight.pause();
                resetLock();
                tagsSeenSummary = "None";
            }
        } catch (Exception ignored) {}
    }

    /** Stops the Limelight and Dashboard stream on OpMode shutdown. */
    public void stop() {
        try {
            FtcDashboard.getInstance().stopCameraStream();
        } catch (Exception ignored) {}
        if (limelight != null) {
            try {
                limelight.stop();
            } catch (Exception ignored) {}
        }
    }
}
