package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.Angles;

import java.util.Locale;

/**
 * PoseProvider respaldado por Road Runner (owner activo hoy). Único archivo del
 * paquete localization que importa tipos de RR: la conversión a doubles ocurre
 * aquí y en ningún otro lado.
 *
 * Calidad publicada: UNINITIALIZED hasta el primer periodic() del
 * DriveSubsystem, después ODOMETRY_ONLY. FUSED_GOOD/DEGRADED llegan con la
 * fusión de MP-04; INVALID queda reservado para fallas detectadas.
 */
public class RoadRunnerPoseProvider implements PoseProvider {

    private final DriveSubsystem driveSubsystem;
    private String lastDiagnostic = "POSE_NOT_UPDATED";

    public RoadRunnerPoseProvider(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public PoseSnapshot getSnapshot() {
        long now = System.nanoTime();
        if (!driveSubsystem.isPoseEverUpdated()) {
            lastDiagnostic = "POSE_NOT_UPDATED";
            return PoseSnapshot.uninitialized(now);
        }

        Pose2d pose = driveSubsystem.getMecanumDrive().pose;
        PoseVelocity2d velocity = driveSubsystem.getLastVelocity();
        double vx = velocity == null ? 0.0 : velocity.linearVel.x;
        double vy = velocity == null ? 0.0 : velocity.linearVel.y;
        double omega = velocity == null ? 0.0 : velocity.angVel;

        double heading = pose.heading.toDouble();
        if (!allFinite(pose.position.x, pose.position.y, heading, vx, vy, omega)) {
            lastDiagnostic = String.format(Locale.US,
                    "NON_FINITE raw x=%s y=%s h=%s vx=%s vy=%s w=%s",
                    pose.position.x, pose.position.y, heading, vx, vy, omega);
            return PoseSnapshot.invalid(now, driveSubsystem.getResetEpoch(),
                    "NON_FINITE_POSE_OR_VELOCITY");
        }

        lastDiagnostic = "OK";

        return PoseSnapshot.of(
                pose.position.x,
                pose.position.y,
                heading,
                vx, vy, omega,
                now,
                driveSubsystem.getResetEpoch(),
                PoseQuality.ODOMETRY_ONLY);
    }

    @Override
    public void setPose(double xInches, double yInches, double headingRadians) {
        trySetPose(xInches, yInches, headingRadians);
    }

    @Override
    public boolean trySetPose(double xInches, double yInches, double headingRadians) {
        if (!allFinite(xInches, yInches, headingRadians)) {
            lastDiagnostic = String.format(Locale.US,
                    "REJECTED_SET_POSE raw x=%s y=%s h=%s",
                    xInches, yInches, headingRadians);
            return false;
        }
        driveSubsystem.setPose(new Pose2d(
                xInches, yInches, Angles.normalizeRadians(headingRadians)));
        lastDiagnostic = "SET_POSE_ACCEPTED";
        return true;
    }

    @Override
    public int getResetEpoch() {
        return driveSubsystem.getResetEpoch();
    }

    @Override
    public String getLastDiagnostic() {
        return lastDiagnostic;
    }

    private static boolean allFinite(double... values) {
        for (double value : values) {
            if (!Double.isFinite(value)) {
                return false;
            }
        }
        return true;
    }
}
