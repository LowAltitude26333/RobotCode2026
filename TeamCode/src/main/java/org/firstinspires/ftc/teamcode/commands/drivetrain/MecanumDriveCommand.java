package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.RunCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class MecanumDriveCommand extends RunCommand {

    public MecanumDriveCommand(DriveSubsystem drive,
                               DoubleSupplier strafe,
                               DoubleSupplier forward,
                               DoubleSupplier turn) {
        super(
                () -> drive.drive(
                        strafe.getAsDouble(),
                        forward.getAsDouble(),
                        turn.getAsDouble()
                         // Por defecto Robot Centric, cámbialo a true si quieres Field Centric
                ),
                drive
        );
    }
}