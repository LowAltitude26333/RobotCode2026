package org.firstinspires.ftc.teamcode.commands.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterHoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootBurstShortCommand extends SequentialCommandGroup {

    public ShootBurstShortCommand(ShooterSubsystem shooter,
                                 ShooterHoodSubsystem hood,
                                 KickerSubsystem kicker,
                                 int shots,
                                 LowAltitudeConstants.TargetRPM targetRpm,
                                 LowAltitudeConstants.HoodPosition targetAngle) {

        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(() -> hood.setPosition(targetAngle), hood),
                        new InstantCommand
                                (() -> shooter.setTargetEnum(targetRpm))
                ),
                new WaitUntilCommand(shooter::onTarget),
                new SequentialCommandGroup(
                        getBurstSequence(kicker, shots, shooter)
                )
        );
    }

    public ShootBurstShortCommand(ShooterSubsystem shooter,
                                 ShooterHoodSubsystem hood,
                                 KickerSubsystem kicker,
                                 int shots) {
        this(shooter, hood, kicker, shots,
                LowAltitudeConstants.TargetRPM.SHORT_SHOT_RPM, // Corregido: Usar el Enum correcto
                LowAltitudeConstants.HoodPosition.SHORT_SHOT);
    }

    // Método auxiliar con la lógica de tiempo variable
    private SequentialCommandGroup getBurstSequence(KickerSubsystem kicker, int shots, ShooterSubsystem shooter) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();

        for (int i = 0; i < shots; i++) {
            // LÓGICA: Si es la primera iteración (i=0), espera 750ms.
            // Si son las siguientes (i=1, 2...), espera 250ms.
            long kickTime = (i == 0) ? 475 : 475;

            sequence.addCommands(
                    new WaitUntilCommand(shooter::onTarget),
                    new InstantCommand(kicker::kick, kicker),
                    new WaitCommand(kickTime), // Tiempo dinámico
                    new InstantCommand(kicker::stop, kicker),
                    new WaitCommand(150)  // Tiempo entre disparos (Recuperación)0


            );
        }
        return sequence;
    }
}