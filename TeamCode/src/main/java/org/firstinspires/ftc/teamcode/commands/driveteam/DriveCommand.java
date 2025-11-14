package org.firstinspires.ftc.teamcode.commands.driveteam;


import static org.firstinspires.ftc.teamcode.LowAltitudeConstants.CHASSIS_POWER;

import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier strafe, forward, turn;

    public DriveCommand(DriveSubsystem subsystem,
                        DoubleSupplier strafe,
                        DoubleSupplier forward,
                        DoubleSupplier turn) {
        this.driveSubsystem = subsystem;
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.drive(
                strafe.getAsDouble() * CHASSIS_POWER,
                forward.getAsDouble() * CHASSIS_POWER,
                turn.getAsDouble()
        );
    }
}