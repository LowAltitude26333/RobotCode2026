package org.firstinspires.ftc.teamcode.commands;

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
}
