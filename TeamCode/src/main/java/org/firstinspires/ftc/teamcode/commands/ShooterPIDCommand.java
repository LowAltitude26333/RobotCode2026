package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShooterPIDCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final double targetRpm;

    public ShooterPIDCommand(ShooterSubsystem shooter, double targetRpm) {
        this.shooter = shooter;
        this.targetRpm = targetRpm;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setTargetRPM(targetRpm);
    }

    @Override
    public void execute() {
        // VACÍO: La magia ocurre en shooter.periodic() automáticamente.
        // Esto asegura que el PID corra siempre, incluso si el comando se interrumpe
        // pero el motor no se apaga explícitamente.
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}