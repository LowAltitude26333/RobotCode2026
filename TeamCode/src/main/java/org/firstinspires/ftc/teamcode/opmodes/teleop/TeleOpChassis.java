package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.AngularCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;

import org.firstinspires.ftc.teamcode.commands.LevnatadorCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterCommadn;
import org.firstinspires.ftc.teamcode.commands .driveteam.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.AngularSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LevnatadorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem2;



@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpNORMAL")
public class TeleOpChassis extends CommandOpMode {

    private IntakeSubsystem intake;
    private ShooterSubsystem2 shooter;

    private LevnatadorSubsystem levantador;

    private AngularSubsystem angulo;



    @Override
    public void initialize() {

        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);

        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        this.intake = new IntakeSubsystem(hardwareMap);
        this.shooter = new ShooterSubsystem2(hardwareMap);
        this.levantador = new LevnatadorSubsystem(hardwareMap);
        this.angulo = new AngularSubsystem(hardwareMap);


        register(drive, intake, shooter, levantador, angulo);

        intake.setDefaultCommand(new IntakeCommand(
                intake,
                () -> driver2.getButton(GamepadKeys.Button.A),
                () -> driver2.getButton(GamepadKeys.Button.B)
        ));
        drive.setDefaultCommand(new DriveCommand(
                drive,
                driver::getLeftX,         // strafe (izquierda/derecha)
                () -> -driver.getLeftY(), // forward/backward (invertido para que arriba sea adelante)
                driver::getRightX
        ));
        shooter.setDefaultCommand(new ShooterCommadn(
                shooter,
                () -> driver2.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        ));

        levantador.setDefaultCommand(new LevnatadorCommand(
                levantador,
                () -> driver2.getButton(GamepadKeys.Button.LEFT_BUMPER),
                () -> driver2.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ));
        angulo.setDefaultCommand(new AngularCommand(
                angulo,
                () -> driver2.getButton(GamepadKeys.Button.X),
                () -> driver2.getButton(GamepadKeys.Button.Y)
        ));
    }
}