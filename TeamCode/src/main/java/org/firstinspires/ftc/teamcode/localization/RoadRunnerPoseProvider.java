package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

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

    public RoadRunnerPoseProvider(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public PoseSnapshot getSnapshot() {
        long now = System.nanoTime();
        if (!driveSubsystem.isPoseEverUpdated()) {
            return PoseSnapshot.uninitialized(now);
        }

        Pose2d pose = driveSubsystem.getMecanumDrive().pose;
        PoseVelocity2d velocity = driveSubsystem.getLastVelocity();
        double vx = velocity == null ? 0.0 : velocity.linearVel.x;
        double vy = velocity == null ? 0.0 : velocity.linearVel.y;
        double omega = velocity == null ? 0.0 : velocity.angVel;

        return new PoseSnapshot(
                pose.position.x,
                pose.position.y,
                pose.heading.toDouble(),
                vx, vy, omega,
                now,
                driveSubsystem.getResetEpoch(),
                PoseQuality.ODOMETRY_ONLY);
    }

    @Override
    public void setPose(double xInches, double yInches, double headingRadians) {
        driveSubsystem.setPose(new Pose2d(xInches, yInches, headingRadians));
    }

    @Override
    public int getResetEpoch() {
        return driveSubsystem.getResetEpoch();
    }
}
