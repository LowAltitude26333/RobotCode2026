package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.function.BooleanSupplier;

public class TurretFollowTagCommand extends CommandBase {
    private final TurretSubsystem turret;
    private final AprilTagProcessor aprilTag;
    private final BooleanSupplier redAllianceSupplier;

    private double lastTrackingPower;
    private long lastDetectionNanos;
    private int lastDesiredId;

    public TurretFollowTagCommand(TurretSubsystem turret,
                                  AprilTagProcessor aprilTag,
                                  BooleanSupplier redAllianceSupplier) {
        this.turret = turret;
        this.aprilTag = aprilTag;
        this.redAllianceSupplier = redAllianceSupplier;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        lastTrackingPower = 0;
        lastDetectionNanos = 0;
        lastDesiredId = -1;
    }

    @Override
    public void execute() {
        if (!turret.isArmed()) {
            turret.stop();
            return;
        }

        int desiredId = redAllianceSupplier.getAsBoolean()
                ? LowAltitudeConstants.TurretConstants.RED_GOAL_TAG_ID
                : LowAltitudeConstants.TurretConstants.BLUE_GOAL_TAG_ID;

        if (desiredId != lastDesiredId) {
            lastTrackingPower = 0;
            lastDetectionNanos = 0;
            lastDesiredId = desiredId;
            turret.stop();
        }

        AprilTagDetection target = null;
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.id == desiredId && detection.ftcPose != null) {
                target = detection;
                break;
            }
        }

        if (target != null) {
            double errorDegrees = -target.ftcPose.bearing;
            double maxPower = LowAltitudeConstants.TurretConstants.TURRET_MAX_POWER;

            if (Math.abs(errorDegrees)
                    <= LowAltitudeConstants.TurretConstants.TURRET_ERROR_TOLERANCE) {
                lastTrackingPower = 0;
                turret.stop();
            } else {
                lastTrackingPower = Math.max(-maxPower, Math.min(maxPower,
                        errorDegrees * LowAltitudeConstants.TurretConstants.TURRET_KP));
                turret.setPower(lastTrackingPower);
            }

            lastDetectionNanos = System.nanoTime();
            return;
        }

        long lossNanos = System.nanoTime() - lastDetectionNanos;
        if (lastDetectionNanos != 0
                && lossNanos <= LowAltitudeConstants.TurretConstants.TAG_LOSS_HOLD_MS * 1_000_000L) {
            turret.setPower(lastTrackingPower);
        } else {
            lastTrackingPower = 0;
            turret.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}
