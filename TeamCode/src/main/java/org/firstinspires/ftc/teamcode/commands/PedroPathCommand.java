package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;

/** Equivalente Pedro de {@link ActionCommand}: sigue un PathChain hasta que Pedro deja de estar busy. */
public class PedroPathCommand extends CommandBase {
    private final PedroDriveSubsystem drive;
    private final PathChain pathChain;

    public PedroPathCommand(PedroDriveSubsystem drive, PathChain pathChain) {
        this.drive = drive;
        this.pathChain = pathChain;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.followPath(pathChain);
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
