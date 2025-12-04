package org.firstinspires.ftc.teamcode.commands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterHoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootBurstCommand extends SequentialCommandGroup {

    public ShootBurstCommand(ShooterSubsystem shooter,
                             ShooterHoodSubsystem hood,
                             KickerSubsystem kicker,
                             int shots,
                             LowAltitudeConstants.TargetRPM targetRpm,
                             LowAltitudeConstants.HoodPosition targetAngle) {

        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(() -> hood.setPosition(targetAngle), hood),
                        new InstantCommand(() -> shooter.setTargetEnum(targetRpm))
                ),
                new WaitUntilCommand(shooter::onTarget),
                new SequentialCommandGroup(
                        getBurstSequence(kicker, shots)
                )
        );
    }

    public ShootBurstCommand(ShooterSubsystem shooter,
                             ShooterHoodSubsystem hood,
                             KickerSubsystem kicker,
                             int shots) {
        this(shooter, hood, kicker, shots,
                LowAltitudeConstants.TargetRPM.MID_FIELD_RPM, // Corregido: Usar el Enum correcto
                LowAltitudeConstants.HoodPosition.MID_FIELD);
    }

    // Método auxiliar con la lógica de tiempo variable
    private SequentialCommandGroup getBurstSequence(KickerSubsystem kicker, int shots) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();

        for (int i = 0; i < shots; i++) {
            // LÓGICA: Si es la primera iteración (i=0), espera 750ms.
            // Si son las siguientes (i=1, 2...), espera 450ms.
            long kickTime = (i == 0) ? 700 : 550;

            sequence.addCommands(
                    new InstantCommand(kicker::kick, kicker),
                    new WaitCommand(kickTime), // Tiempo dinámico
                    new InstantCommand(kicker::stop, kicker),
                    new WaitCommand(750)  // Tiempo entre disparos (Recuperación)
            );
        }
        return sequence;
    }
}