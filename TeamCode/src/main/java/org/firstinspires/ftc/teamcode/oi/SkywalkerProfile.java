package org.firstinspires.ftc.teamcode.oi;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.commands.ShooterPIDCommand;

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

        // RESET GIROSCOPIO (Usando el método seguro del subsistema)
        new GamepadButton(driverOp, GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(robot.driveSubsystem::resetHeading));


        // =================================================================
        // TOOL OP (Gamepad 2) - Pruebas Manuales
        // =================================================================

        // --- 1. HOOD (Servos) ---
        // D-Pad Arriba -> Tiro Lejano
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() ->
                        robot.hoodSubsystem.setPosition(LowAltitudeConstants.HoodPosition.LONG_SHOT), robot.hoodSubsystem));

        // D-Pad Abajo -> Tiro Pared
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() ->
                        robot.hoodSubsystem.setPosition(LowAltitudeConstants.HoodPosition.WALL_SHOT), robot.hoodSubsystem));

        // D-Pad Derecha -> Ángulo MEDIO
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() ->
                        robot.hoodSubsystem.setPosition(LowAltitudeConstants.HoodPosition.MID_FIELD), robot.hoodSubsystem));

        //D-Pad Izquierda -> ángulo 0
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() ->
                        robot.hoodSubsystem.setPosition(LowAltitudeConstants.HoodPosition.HOME_POS), robot.hoodSubsystem));

        // --- 2. SHOOTER (Motor PIDF) ---

    // Y -> Prender Shooter y MANTENER velocidad (4500 RPM)
    // Este comando corre indefinidamente hasta que otro comando use el shooter
        new GamepadButton(toolOp, GamepadKeys.Button.Y)
                .whenPressed(new ShooterPIDCommand(robot.shooterSubsystem, LowAltitudeConstants.SHOOTER_SPEED_RPM));

    // B -> Prender Shooter Velocidad Baja (2000 RPM)
    // Esto interrumpirá al de 4500 y pondrá el nuevo de 2000
        new GamepadButton(toolOp, GamepadKeys.Button.B)
                .whenPressed(new ShooterPIDCommand(robot.shooterSubsystem, LowAltitudeConstants.SHOOTER_LOW_SPEED_RPM));

    // A -> Apagar Shooter (Stop)
    // Al requerir el subsistema, este comando mata al PIDCommand activo.
        new GamepadButton(toolOp, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(
                        robot.shooterSubsystem::stop, // Acción
                        robot.shooterSubsystem        // Requisito (CRÍTICO para que funcione el apagado)
                ));

        // --- 3. KICKER (Motor Raw Power) ---
        // RB (Hold) -> Activar Kicker
        new GamepadButton(toolOp, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(robot.kickerSubsystem::kick, robot.kickerSubsystem))
                .whenReleased(new InstantCommand(robot.kickerSubsystem::stop, robot.kickerSubsystem));

        // LB (Hold) -> Reversa Kicker
        new GamepadButton(toolOp, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new RunCommand(robot.kickerSubsystem::reverse, robot.kickerSubsystem))
                .whenReleased(new InstantCommand(robot.kickerSubsystem::stop, robot.kickerSubsystem));

        // 4 ----- INTAKE -------
        // RB -> Intake Adentro (Comer)
        new GamepadButton(toolOp, GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(robot.intakeSubsystem::intakeOn, robot.intakeSubsystem));
        /*
        new GamepadButton(toolOp, GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(robot.intakeSubsystem::intakeOn, robot.intakeSubsystem));
         */

        // LB -> Intake Afuera (Escupir)
        new GamepadButton(toolOp, GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(robot.intakeSubsystem::intakeReturn, robot.intakeSubsystem));

        // A -> Intake Apagado
        new GamepadButton(toolOp, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(robot.intakeSubsystem::intakeOff, robot.intakeSubsystem));
    }

}