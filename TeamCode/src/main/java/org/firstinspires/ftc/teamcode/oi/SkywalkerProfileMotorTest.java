package org.firstinspires.ftc.teamcode.oi;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.commands.ShooterPIDCommand;

public class SkywalkerProfileMotorTest implements ControlProfile {

    private final GamepadEx driverOp;
    private final GamepadEx toolOp;

    public SkywalkerProfileMotorTest(Gamepad gamepad1, Gamepad gamepad2) {
        this.driverOp = new GamepadEx(gamepad1);
        this.toolOp = new GamepadEx(gamepad2);
    }

    @Override
    public GamepadEx getDriverOp() { return driverOp; }

    @Override
    public GamepadEx getToolOp() { return toolOp; }

    // --- CHASIS ---
    @Override
    public double getDriveStrafe() { return driverOp.getLeftX(); }
    @Override
    public double getDriveForward() { return driverOp.getLeftY(); }
    @Override
    public double getDriveTurn() { return driverOp.getRightX(); }

    @Override
    public void configureButtonBindings(RobotContainer robot) {

        /*
         * =================================================================
         * DRIVER (PILOTO)
         * Enfoque: Posicionamiento y Disparo Final
         * =================================================================
         */

        // 1. DISPARO (Gatillo) - El driver decide cuándo disparar
        // RB (Hold) -> Activar Kicker
        new GamepadButton(driverOp, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(robot.kickerSubsystem::kick, robot.kickerSubsystem))
                .whenReleased(new InstantCommand(robot.kickerSubsystem::stop, robot.kickerSubsystem));

        // 2. RESET GIROSCOPIO (Start)
        new GamepadButton(driverOp, GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(robot.driveSubsystem::resetHeading));

        // 3. GLOBAL EMERGENCY STOP (Back)
        // "Rómpase en caso de emergencia": Mata comandos y frena todo.
        new GamepadButton(driverOp, GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> {
                    CommandScheduler.getInstance().cancelAll(); // Cancela PID, trayectorias, etc.
                    robot.driveSubsystem.stop();
                    robot.shooterSubsystem.stop();
                    robot.intakeSubsystem.intakeOff();
                    robot.kickerSubsystem.stop();
                }));


        /*
         * =================================================================
         * TOOL OP (ARTILLERO)
         * Enfoque: Configuración de Estado (State Machine)
         * =================================================================
         */

        // --- PRESETS DE DISPARO (Hood + RPM Simultáneos) ---

        // A -> WALL SHOT (Cerca)
        new GamepadButton(toolOp, GamepadKeys.Button.A)
                .whenPressed(new ParallelCommandGroup(
                        new InstantCommand(() -> robot.hoodSubsystem.setPosition(LowAltitudeConstants.HoodPosition.
                                WALL_SHOT), robot.hoodSubsystem),
                        new ShooterPIDCommand(robot.shooterSubsystem, LowAltitudeConstants.TargetRPM.WALL_SHOT_RPM.targetRPM)
                ));

        // X -> SHORT SHOT (Corto alcance / Stack)
        new GamepadButton(toolOp, GamepadKeys.Button.X)
                .whenPressed(new ParallelCommandGroup(
                        new InstantCommand(() -> robot.hoodSubsystem.setPosition(LowAltitudeConstants.HoodPosition.SHORT_SHOT), robot.hoodSubsystem),
                        new ShooterPIDCommand(robot.shooterSubsystem, LowAltitudeConstants.TargetRPM.SHORT_SHOT_RPM.targetRPM)
                ));

        // B -> MID FIELD (Media Cancha)
        new GamepadButton(toolOp, GamepadKeys.Button.B)
                .whenPressed(new ParallelCommandGroup(
                        new InstantCommand(() -> robot.hoodSubsystem.setPosition(LowAltitudeConstants.HoodPosition.MID_FIELD), robot.hoodSubsystem),
                        new ShooterPIDCommand(robot.shooterSubsystem, LowAltitudeConstants.TargetRPM.MID_FIELD_RPM.targetRPM)
                ));

        // Y -> LONG SHOT (Lejos)
        new GamepadButton(toolOp, GamepadKeys.Button.Y)
                .whileHeld(new RunCommand(() -> robot.shooterSubsystem.driveShooter(0.9))
                );



        // --- INTAKE ---
        // RB -> Intake ON
        new GamepadButton(toolOp, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(robot.intakeSubsystem::intakeOn, robot.intakeSubsystem));

        // LB -> Intake REVERSE
        new GamepadButton(toolOp, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(robot.intakeSubsystem::intakeReturn, robot.intakeSubsystem));

        // D-Pad Down -> Intake OFF + Shooter OFF (Modo "Quiet")
        // Útil para apagar todo el ruido cuando ya no se dispara
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new ParallelCommandGroup(
                        new InstantCommand(robot.intakeSubsystem::intakeOff, robot.intakeSubsystem),
                        new InstantCommand(robot.shooterSubsystem::stop, robot.shooterSubsystem)
                ));


        // --- UTILIDADES MANUALES (Backup) ---

        // D-Pad Up -> Kicker Manual (Por si el driver falla o se ocupa)
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_UP)
                .whileHeld(new RunCommand(robot.kickerSubsystem::kick, robot.kickerSubsystem))
                .whenReleased(new InstantCommand(robot.kickerSubsystem::stop, robot.kickerSubsystem));

        // D-Pad Left -> Kicker Retract (Desatascar)
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_LEFT)
                .whileHeld(new RunCommand(robot.kickerSubsystem::reverse, robot.kickerSubsystem))
                .whenReleased(new InstantCommand(robot.kickerSubsystem::stop, robot.kickerSubsystem));

        //F-Pad Right -> Hood 0
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> robot.hoodSubsystem.setPosition(LowAltitudeConstants.HoodPosition.
                        HOME_POS), robot.hoodSubsystem));
    }
}