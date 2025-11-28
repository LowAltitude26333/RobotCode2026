package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShooterPIDCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final double targetRpm;

    public ShooterPIDCommand(ShooterSubsystem shooter, double targetRpm) {
        this.shooter = shooter;
        this.targetRpm = targetRpm;

        // REQUISITO CRÍTICO: Al requerir el subsistema, este comando
        // cancelará cualquier otro comando que esté usando el shooter.
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // Al empezar, le decimos al subsistema cuál es la meta
        shooter.setTargetRPM(targetRpm);
    }

    @Override
    public void execute() {
        // Ciclo a ciclo (loop), corremos la matemática del PID
        shooter.runPID();
    }

    @Override
    public void end(boolean interrupted) {
        // Si el comando termina (o es interrumpido por el botón A),
        // apagamos el motor por seguridad.
        shooter.stop();
    }
}