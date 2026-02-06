package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants; // Importar Constantes
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterHoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootSequenceCommand extends SequentialCommandGroup {

    public ShootSequenceCommand(ShooterSubsystem shooter,
                                ShooterHoodSubsystem hood,
                                KickerSubsystem kicker,
                                double targetRpm,
                                LowAltitudeConstants.HoodPosition angle) {

        addCommands(
                // 1. Preparación (Hood y RPM al mismo tiempo)
                new ParallelCommandGroup(
                        new InstantCommand(() -> hood.setPosition(angle), hood),
                        new InstantCommand(() -> shooter.setTargetRPM(targetRpm))
                ),

                // 2. Esperar a que el motor llegue a la velocidad (Ready to fire)
                new WaitUntilCommand(shooter::isReady),

                // 3. DISPARO MECÁNICO (Raw Power)
                // Encender motor del kicker
                new InstantCommand(kicker::kick, kicker),

                // Esperar lo suficiente para que el mecanismo empuje el anillo
                // Al ser Raw Power, esto es prueba y error. Empieza con 150ms.
                new WaitCommand(150),

                // Apagar motor (el freno configurado en el subsistema lo detendrá)
                new InstantCommand(kicker::stop, kicker)
        );
    }
}