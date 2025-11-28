package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.teamcode.subsystems.AngularSubsystem;

import java.util.function.BooleanSupplier;



public class AngularCommand extends CommandBase {

    private final AngularSubsystem slideSubsystem;
    private final BooleanSupplier position1Button;
    private final BooleanSupplier position2Button;




    public AngularCommand(AngularSubsystem slideSubsystem,
                          BooleanSupplier position1Button,
                          BooleanSupplier position2Button) {
        this.slideSubsystem = slideSubsystem;
        this.position1Button = position1Button;
        this.position2Button = position2Button;
        addRequirements(slideSubsystem);
    }

    @Override
    public void execute() {
        if (position1Button.getAsBoolean()) {
            slideSubsystem.position1(); // Expandir
        } else if (position2Button.getAsBoolean()) {
            slideSubsystem.position2(); // Contraer
        }
        else {
            slideSubsystem.stop(); // Mantener
        }
    }

    @Override
    public void end(boolean interrupted){
        slideSubsystem.stop();
    }
}
