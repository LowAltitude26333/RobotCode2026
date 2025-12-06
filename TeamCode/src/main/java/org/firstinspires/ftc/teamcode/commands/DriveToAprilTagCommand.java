package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.Supplier;

public class DriveToAprilTagCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final Supplier<AprilTagDetection> detectionSupplier;

    private final int targetId;

    private final int targetId2;


    public DriveToAprilTagCommand(DriveSubsystem driveSubsystem,Supplier<AprilTagDetection> detectionSupplier,
                                  int targetId, int targetId2) {
        this.driveSubsystem = driveSubsystem;
        this.detectionSupplier = detectionSupplier;
        this.targetId = targetId;
        this.targetId2 = targetId2;
        addRequirements(driveSubsystem);
    }

    private AprilTagDetection getTargetTag() {
        AprilTagDetection det = detectionSupplier.get();

        if (det == null) {
            return null;
        }

        if (det.id == targetId || det.id == targetId2) {
            return det;
        }

        return null;
    }
    @Override
    public void execute() {
        AprilTagDetection det = getTargetTag();
        if (det == null) {
            driveSubsystem.stop();
            return;
        }

        double yaw = det.ftcPose.yaw;

        double turnPower = yaw * 0.4;
        turnPower =
                Math.max(-0.4, Math.min(0.4, turnPower));

        driveSubsystem.drive(0, 0, turnPower);
    }

    @Override
    public boolean isFinished() {
        AprilTagDetection det = getTargetTag();
        return det == null || Math.abs(det.ftcPose.yaw) < 2;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
}