package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

public class TurretFollowTagCommand extends CommandBase {

    // VARIABLES GLOBALES (Esto es lo que le falta a tu código para que 'end' y 'execute' lo reconozcan)
    private final TurretSubsystem turret;
    private final AprilTagProcessor aprilTag;
    private double searchDirection = 1.0;

    // CONSTRUCTOR
    public TurretFollowTagCommand(TurretSubsystem turret, AprilTagProcessor aprilTag) {
        this.turret = turret;
        this.aprilTag = aprilTag;
        addRequirements(turret); // Le dice al Scheduler que este comando usa la torreta
    }

    @Override
    public void execute() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (!detections.isEmpty()) {
            // --- MODO TRACKING (Viendo AprilTag) ---
            AprilTagDetection target = detections.get(0);

            // Signo negativo (-) corregido para que apunte en la dirección correcta
            double error = -target.ftcPose.bearing;

            searchDirection = Math.signum(error);

            if (Math.abs(error) > LowAltitudeConstants.TurretConstants.TURRET_ERROR_TOLERANCE) {
                double power = error * LowAltitudeConstants.TurretConstants.TURRET_KP;
                double max = LowAltitudeConstants.TurretConstants.TURRET_MAX_POWER;

                turret.setPower(Math.max(-max, Math.min(max, power)));
            } else {
                turret.stop();
            }

        } else {
            // --- MODO BÚSQUEDA CON REBOTE (No hay AprilTag) ---
            double currentPos = turret.getPosition();

            if (currentPos <= TurretSubsystem.LIMIT_LEFT + 10) {
                searchDirection = 1.0;
            } else if (currentPos >= TurretSubsystem.LIMIT_RIGHT - 10) {
                searchDirection = -1.0;
            }

            // Potencia de búsqueda constante y suave
            turret.setPower(searchDirection * 0.3);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Al estar declarada arriba como variable global, 'turret' ya funciona aquí sin problemas
        turret.stop();
    }
}