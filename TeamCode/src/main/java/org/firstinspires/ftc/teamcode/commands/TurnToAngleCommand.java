package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class TurnToAngleCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final DoubleSupplier strafe, forward;
    private final DoubleSupplier targetAngleSupplier;
    private final PIDController pid;

    // --- TUNING POWERHOUSE ---
    // P: Bajamos a 0.02 para reducir la agresión inicial (menos overshoot).
    // D: Subimos a 0.003 para que actúe como "freno" al acercarse.
    // I: 0 (Casi nunca se usa en giros de chasis).
    private static final double P = 0.022, I = 0.0, D = 0.0035;

    // Tolerancia: Si el error es menor a 1.5 grados, cortamos potencia para no vibrar.
    private static final double ANGLE_TOLERANCE = 3.5;

    public TurnToAngleCommand(DriveSubsystem drive, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier targetAngleSupplier) {
        this.drive = drive;
        this.strafe = strafe;
        this.forward = forward;
        this.targetAngleSupplier = targetAngleSupplier;

        this.pid = new PIDController(P, I, D);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double target = targetAngleSupplier.getAsDouble();
        double currentHeading = Math.toDegrees(drive.getHeading());

        // Calcular error más corto (-180 a 180)
        double error = target - currentHeading;
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;

        double turnPower = 0;

        if (Math.abs(error) > ANGLE_TOLERANCE) {
            turnPower = pid.calculate(0, error);

            // CLAMP DE SEGURIDAD:
            // Limitamos la velocidad máxima de giro al 70% para evitar descontrol
            turnPower = Math.max(-0.7, Math.min(0.7, turnPower));
        }

        // --- CORRECCIÓN CRÍTICA ---
        // Tu DriveSubsystem invierte el giro internamente (linea -turnSpeed).
        // Por lo tanto, aquí debemos INVERTIRLO también para que (-) con (-) dé (+).
        // Si antes giraba al revés, esto lo arregla.
        drive.drive(strafe.getAsDouble(), forward.getAsDouble(), -turnPower);
    }

    @Override
    public boolean isFinished() {
        return false; // Vive mientras se presione el botón (whileHeld)
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0);
    }
}