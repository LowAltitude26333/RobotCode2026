package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commands.PoseStorage;

public class SetPrecisionModeCommand extends CommandBase {

    @Override
    public void initialize() {
        // Al presionar el botón, activamos modo precisión
        PoseStorage.isPrecisionMode = true;
    }

    @Override
    public boolean isFinished() {
        // Retornamos false para que el comando siga "vivo" mientras se mantenga el botón
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Al soltar el botón (interrupción), desactivamos modo precisión
        PoseStorage.isPrecisionMode = false;
    }
}