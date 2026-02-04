package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class TurnToAngleCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final double targetAngle;
    private final PIDController pid;

    // Ajusta estos valores: P es la fuerza de reacción, D frena la oscilación
    private static final double P = 0.03, I = 0, D = 0.001;

    public TurnToAngleCommand(DriveSubsystem drive, double targetAngleDegrees) {
        this.drive = drive;
        this.targetAngle = targetAngleDegrees;
        this.pid = new PIDController(P, I, D);
        // El error puede ir de -180 a 180, esto le enseña al PID el camino más corto
        // NOTA: FTCLib PID no tiene continuousInput nativo simple, hay que normalizar el error manualmente
        // o usar el profile de RoadRunner, pero para teleop simple esto basta.
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double currentHeading = Math.toDegrees(drive.getHeading());
        double error = targetAngle - currentHeading;

        // Normalizar error a -180 a 180 para tomar el camino corto
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;

        double turnPower = pid.calculate(0, error); // Queremos que el error sea 0

        // Mantenemos el control manual de X e Y, solo secuestramos el giro
        drive.drive(0, 0, turnPower);
        // OJO: Si quieres moverte mientras giras, necesitarías pasar los joysticks al constructor
    }

    @Override
    public boolean isFinished() {
        // Opcional: Terminar si el error es pequeño (< 2 grados)
        // return Math.abs(targetAngle - Math.toDegrees(drive.getHeading())) < 2;
        return false; // En TeleOp solemos mantenerlo mientras se presiona el botón
    }
}