/*package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeOn extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public IntakeOn(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        intakeSubsystem.intakeOn();
    }
    public void end(){
        intakeSubsystem.intakeOff();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}*/
package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.BooleanSupplier;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final BooleanSupplier comerButton;
    private final BooleanSupplier escupirButton;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, BooleanSupplier comerButton, BooleanSupplier escupirButton) {
        this.intakeSubsystem = intakeSubsystem;
        this.comerButton = comerButton;
        this.escupirButton = escupirButton;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (comerButton.getAsBoolean()) {
            intakeSubsystem.intakeOn();
        } else if (escupirButton.getAsBoolean()) {
            intakeSubsystem.intakeReturn();
        } else {
            intakeSubsystem.intakeOff();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.intakeOff();
    }
}

