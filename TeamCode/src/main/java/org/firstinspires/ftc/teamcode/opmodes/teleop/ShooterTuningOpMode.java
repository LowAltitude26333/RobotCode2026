package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.SafeCommandOpMode;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Config
@TeleOp(name = "Tuning: Shooter & Systems (Manual)", group = "Tuning")
public class ShooterTuningOpMode extends SafeCommandOpMode {

    // --- VARIABLES EDITABLES EN DASHBOARD ---
    // Ajustado a tu máximo real aprox para evitar saturación del PID
    public static double TARGET_RPM = 3500;

    private ShooterSubsystem shooter;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;

    private GamepadEx gamepad;
    private boolean shooterEnabled;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        gamepad = new GamepadEx(gamepad1);

        // --- CONTROLES ---

        // 1. SHOOTER PID (Y -> On | A -> Off)
        new GamepadButton(gamepad, GamepadKeys.Button.Y)
                .whileHeld(new RunCommand(() -> {
                    shooterEnabled = true;
                    shooter.setTargetRPM(TARGET_RPM);
                }, shooter))
                .whenReleased(new InstantCommand(() -> {
                    shooterEnabled = false;
                    shooter.stop();
                }, shooter));

        new GamepadButton(gamepad, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> {
                    shooterEnabled = false;
                    shooter.stop();
                }, shooter));

        // 3. INTAKE
        new GamepadButton(gamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(intake::intakeOn, intake))
                .whenReleased(new InstantCommand(intake::intakeOff, intake));
        new GamepadButton(gamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new RunCommand(intake::intakeReturn, intake))
                .whenReleased(new InstantCommand(intake::intakeOff, intake));
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(intake::intakeOff, intake));

        // 4. KICKER
        new GamepadButton(gamepad, GamepadKeys.Button.X)
                .whileHeld(new RunCommand(kicker::kick, kicker))
                .whenReleased(new InstantCommand(kicker::stop, kicker));
        new GamepadButton(gamepad, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(kicker::stop, kicker));
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP)
                .whileHeld(new RunCommand(kicker::reverse, kicker))
                .whenReleased(new InstantCommand(kicker::stop, kicker));

        telemetry.addLine("--- LISTO PARA TUNING ---");
        telemetry.update();
    }

    @Override
    public void run() {
        // TRUCO DE TUNING EN VIVO
        // Actualiza el target del subsistema si cambias la variable en Dashboard
        // y el motor ya está corriendo (target != 0)
        shooter.reloadControllersIfConstantsChanged();
        if (shooterEnabled && Double.compare(shooter.getTargetRPM(), TARGET_RPM) != 0) {
            shooter.setTargetRPM(TARGET_RPM);
        }

        super.run();

        telemetry.addData(">> DASHBOARD <<", "Edita TARGET_RPM");
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Actual RPM", shooter.getActualShooterRPM());
        telemetry.addData("Shooter Enabled", shooterEnabled);
        telemetry.addData("Kicker servo solicitado", kicker.isServoEnabledAtInit());
        telemetry.addData("Kicker servo disponible", kicker.isServoAvailable());
        telemetry.addData("Kicker servo activo", kicker.isServoActive());
        telemetry.update();
    }
}
