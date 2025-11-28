package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.LevnatadorSubsystem;

import java.util.function.BooleanSupplier;

public class LevnatadorCommand extends CommandBase {

    private final LevnatadorSubsystem levantador;

    private final BooleanSupplier lev2;       // botón para subir (left2)
    private final BooleanSupplier revButton;  // botón para bajar (reverse)

    private boolean levantadorUpOn = false;
    private boolean levantadorDownOn = false;

    private boolean lastLev2State = false;
    private boolean lastRevState = false;

    public LevnatadorCommand(LevnatadorSubsystem levantador,
                             BooleanSupplier lev2Button,
                             BooleanSupplier revButton) {

        this.levantador = levantador;
        this.lev2 = lev2Button;
        this.revButton = revButton;

        addRequirements(levantador);
    }

    @Override
    public void execute() {

        boolean currentLev2 = lev2.getAsBoolean();
        boolean currentRev = revButton.getAsBoolean();

        // ---- Toggle subida ----
        if (currentLev2 && !lastLev2State) {
            levantadorUpOn = !levantadorUpOn;
            // si activas subir, apaga reverse
            if (levantadorUpOn) levantadorDownOn = false;
        }

        // ---- Toggle bajada ----
        if (currentRev && !lastRevState) {
            levantadorDownOn = !levantadorDownOn;
            // si activas bajar, apaga left2
            if (levantadorDownOn) levantadorUpOn = false;
        }

        // ---- Prioridades ----
        if (levantadorUpOn) {
            levantador.left2();
        } else if (levantadorDownOn) {
            levantador.reverse();
        } else {
            levantador.stop();
        }

        lastLev2State = currentLev2;
        lastRevState = currentRev;
    }

    @Override
    public void end(boolean interrupted) {
        levantadorUpOn = false;
        levantadorDownOn = false;
        levantador.stop();
    }
}
