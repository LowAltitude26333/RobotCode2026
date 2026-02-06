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

    // Constructor Principal
    public ShootBurstCommand(ShooterSubsystem shooter,
                             ShooterHoodSubsystem hood,
                             KickerSubsystem kicker,
                             int shots,
                             LowAltitudeConstants.TargetRPM targetRpm,
                             LowAltitudeConstants.HoodPosition targetAngle) {

        addCommands(
                // 1. Preparar Sistemas (Shooter y Hood al mismo tiempo)
                new ParallelCommandGroup(
                        // Ajustar ángulo del hood (usando el valor double del enum)
                        new InstantCommand(() -> hood.setAngle(targetAngle.angle), hood),
                        // Encender Shooter
                        new InstantCommand(() -> shooter.setTargetEnum(targetRpm), shooter)
                ),

                // 2. Esperar a que el Shooter llegue a la velocidad objetivo por primera vez
                new WaitUntilCommand(shooter::isReady),

                // 3. Ejecutar la ráfaga de disparos
                getBurstSequence(kicker, shooter, shots)
        );
    }

    // Constructor Simplificado (Overload)
    public ShootBurstCommand(ShooterSubsystem shooter,
                             ShooterHoodSubsystem hood,
                             KickerSubsystem kicker,
                             int shots) {
        this(shooter, hood, kicker, shots,
                LowAltitudeConstants.TargetRPM.SHORT_SHOT_RPM,
                LowAltitudeConstants.HoodPosition.SHORT_SHOT);
    }

    /**
     * Genera la secuencia de movimientos del Kicker (Gatillo).
     * Incluye verificación de RPM entre cada disparo para máxima precisión.
     */
    private SequentialCommandGroup getBurstSequence(KickerSubsystem kicker, ShooterSubsystem shooter, int shots) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();

        for (int i = 0; i < shots; i++) {
            // Tiempos de empuje del servo
            // Nota: 632ms es bastante tiempo para un servo, asegúrate que no sea el ciclo total.
            // Si el kicker es rápido, podrías bajar esto a 200-300ms para disparar más rápido.
            long extendTime = 632;
            long retractTime = 60; // Tiempo para volver atrás

            sequence.addCommands(
                    // A. Validar RPM (Recuperación Bang-Bang)
                    // Si el disparo anterior bajó mucho la velocidad, esperamos aquí.
                    new WaitUntilCommand(shooter::isReady),

                    // B. Disparar (Empujar)
                    new InstantCommand(kicker::kick, kicker),
                    new WaitCommand(extendTime),

                    // C. Retraer
                    new InstantCommand(kicker::stop, kicker), // O kicker::retract si tienes ese método
                    new WaitCommand(retractTime)
            );
        }
        return sequence;
    }
}