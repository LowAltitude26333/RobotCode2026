package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.driveteam.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpChassis")
public class TeleOpChassis extends CommandOpMode {

    @Override
    public void initialize() {

        GamepadEx driver = new GamepadEx(gamepad1);

        DriveSubsystem drive = new DriveSubsystem(hardwareMap);


        register(drive);


        drive.setDefaultCommand(new DriveCommand(
                drive,
                () -> driver.getLeftX(),        // strafe (izquierda/derecha)
                () -> -driver.getLeftY(), // forward/backward (invertido para que arriba sea adelante)
                driver::getRightX
        ));

    }
}