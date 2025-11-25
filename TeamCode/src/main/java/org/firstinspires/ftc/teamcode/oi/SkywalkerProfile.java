package org.firstinspires.ftc.teamcode.oi;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotContainer;

public class SkywalkerProfile implements ControlProfile {

    private final GamepadEx driverOp;
    private final GamepadEx toolOp;

    public SkywalkerProfile(Gamepad gamepad1, Gamepad gamepad2) {
        this.driverOp = new GamepadEx(gamepad1);
        this.toolOp = new GamepadEx(gamepad2);
    }

    @Override
    public GamepadEx getDriverOp() { return driverOp; }

    @Override
    public GamepadEx getToolOp() { return toolOp; }

    // --- CHASIS (Arcade Drive estándar) ---
    @Override
    public double getDriveStrafe() { return driverOp.getLeftX(); }
    @Override
    public double getDriveForward() { return driverOp.getLeftY(); }
    @Override
    public double getDriveTurn() { return driverOp.getRightX(); }

    @Override
    public void configureButtonBindings(RobotContainer robot) {

        // =================================================================
        // DRIVER (Gamepad 1) - Movilidad y Recolección
        // =================================================================

        // INTAKE (Prueba básica de encendido/apagado/reversa)
        // RB -> Intake Adentro (Comer)
        new GamepadButton(driverOp, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(robot.intakeSubsystem::intakeOn, robot.intakeSubsystem));

        // LB -> Intake Afuera (Escupir)
        new GamepadButton(driverOp, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(robot.intakeSubsystem::intakeReturn, robot.intakeSubsystem));

        // A -> Intake Apagado (Parar todo)
        new GamepadButton(driverOp, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(robot.intakeSubsystem::intakeOff, robot.intakeSubsystem));

        // RESET GIROSCOPIO (Importante para probar Field Centric si lo usas después)
        new GamepadButton(driverOp, GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(robot.driveSubsystem::resetHeading));


        // =================================================================
        // TOOL OP (Gamepad 2) - Pruebas de Sistemas de Disparo
        // =================================================================

        // --- 1. HOOD (Servos) ---
        // Probamos los límites físicos definidos en las constantes

        // D-Pad Arriba -> Ir a ángulo MÁXIMO (Tiro Lejano)
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() ->
                        robot.hoodSubsystem.setPosition(LowAltitudeConstants.HoodPosition.LONG_SHOT), robot.hoodSubsystem));

        // D-Pad Abajo -> Ir a ángulo MÍNIMO (Tiro Pared)
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() ->
                        robot.hoodSubsystem.setPosition(LowAltitudeConstants.HoodPosition.WALL_SHOT), robot.hoodSubsystem));

        // D-Pad Derecha -> Ángulo MEDIO (Para verificar punto intermedio)
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() ->
                        robot.hoodSubsystem.setPosition(LowAltitudeConstants.HoodPosition.MID_FIELD), robot.hoodSubsystem));


        // --- 2. SHOOTER (Motor PIDF) ---
        // Probamos si el motor gira, si hace ruido, y si el PID mantiene la velocidad

        // Y -> Prender Shooter (Velocidad Alta de prueba)
        new GamepadButton(toolOp, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> robot.shooterSubsystem.setCurrentTarget(4500))); // Ajustar RPM

        // B -> Prender Shooter (Velocidad Baja/Idle)
        new GamepadButton(toolOp, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> robot.shooterSubsystem.setCurrentTarget(2000)));

        // A -> Apagar Shooter (Stop)
        new GamepadButton(toolOp, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(robot.shooterSubsystem::stopMotors));


        // --- 3. KICKER (Motor Raw Power) ---
        // Probamos el mecanismo de empuje manualmente.
        // Mientras mantienes presionado el gatillo o bumper, el motor gira.
        // Al soltar, se detiene (freno).

        // Right Bumper (Hold) -> Activar Kicker (Golpear)
        // Usamos RunCommand para que se ejecute continuamente mientras se presiona
        new GamepadButton(toolOp, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(robot.kickerSubsystem::kick, robot.kickerSubsystem))
                .whenReleased(new InstantCommand(robot.kickerSubsystem::stop, robot.kickerSubsystem));

        // Left Bumper (Hold) -> Reversa Kicker (Por si se atora)
        new GamepadButton(toolOp, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new RunCommand(robot.kickerSubsystem::retract, robot.kickerSubsystem))
                .whenReleased(new InstantCommand(robot.kickerSubsystem::stop, robot.kickerSubsystem));
    }
}