package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterHoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterMotor;
import org.firstinspires.ftc.teamcode.commands.ShooterCommand2;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ShooterTeleOpAdaptado")
public class TeleOpShooter extends CommandOpMode {

    private ShooterMotor shooter;
    private ShooterHoodSubsystem hood;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;

    @Override
    public void initialize() {

        // Gamepads

        GamepadEx drivertwo = new GamepadEx(gamepad2);

        // Subsystems
        this.shooter = new ShooterMotor(hardwareMap, telemetry);
        hood = new ShooterHoodSubsystem(hardwareMap, telemetry);
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);


        // Registrar subsistemas
        register(shooter);

        // Shooter con toggle en botón LB de gamepad2
        shooter.setDefaultCommand(new ShooterCommand2(
                shooter,
                () -> drivertwo.getButton(GamepadKeys.Button.A)
        ));

        new GamepadButton(drivertwo, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(intake::intakeOn, intake));
        new GamepadButton(drivertwo, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(intake::intakeReturn, intake));
        new GamepadButton(drivertwo, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(intake::intakeOff, intake));

        // 4. KICKER
        new GamepadButton(drivertwo, GamepadKeys.Button.X)
                .whileHeld(new RunCommand(kicker::kick, kicker))
                .whenReleased(new InstantCommand(kicker::stop, kicker));
        new GamepadButton(drivertwo, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(kicker::stop, kicker));
        new GamepadButton(drivertwo, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(kicker::reverse, kicker));


        // Aquí puedes agregar otros subsistemas si los necesitas
        // Ejemplo:
        // DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        // register(drive);
        // drive.setDefaultCommand(new DriveCommand(...));
    }
}
