package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ColorSubsystem;

public class ColorDetectCommand extends CommandBase {

    private final ColorSubsystem colorSubsystem;

    public ColorDetectCommand(ColorSubsystem subsystem) {
        this.colorSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        colorSubsystem.getColor();

    }

    @Override
    public boolean isFinished() {
        return false; // correr siempre
    }
}