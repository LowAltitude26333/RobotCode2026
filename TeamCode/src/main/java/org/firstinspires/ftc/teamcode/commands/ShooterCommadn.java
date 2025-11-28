package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem2;

import java.util.function.BooleanSupplier;

public class ShooterCommadn extends CommandBase {
    private final ShooterSubsystem2 shooter2;
    private final BooleanSupplier right2;
    private boolean shooterOn = false;
    private boolean lastRight2State = false;

    public ShooterCommadn(ShooterSubsystem2 shooter2, BooleanSupplier right2Button) {
        this.shooter2 = shooter2;
        this.right2 = right2Button;
        addRequirements(shooter2);
    }

    @Override
    public void execute() {
        boolean currentRight2 = right2.getAsBoolean();

        // Toggle: activa o desactiva el shooter con cada pulsación
        if (currentRight2 && !lastRight2State) {
            shooterOn = !shooterOn;
        }

        // Ejecuta según estado
        if (shooterOn) {
            shooter2.left2();  // ← usa left2() si esa es la dirección correcta
        } else {
            shooter2.stop();
        }
        // Actualiza estado anterior
        lastRight2State = currentRight2;
    }

    @Override
    public void end(boolean interrupted) {
        shooterOn = false;
        shooter2.stop();
    }
}