package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class DriveSubsystem extends SubsystemBase {

    private final MecanumDrive drive;
    private double speedMultiplier = 1.0;

    public DriveSubsystem(HardwareMap hardwareMap, Pose2d startPose) {
        drive = new MecanumDrive(hardwareMap, startPose);
    }

    /**
     * Método principal de manejo
     */
    public void drive(double strafe, double forward, double turn, boolean isFieldCentric) {

        // Aplicar multiplicador (Slow Mode)
        double x = strafe * speedMultiplier;
        double y = forward * speedMultiplier;
        double t = turn * speedMultiplier;

        if (isFieldCentric) {
            // CORRECCIÓN: Accedemos al heading a través del localizer
            double heading = drive.localizer.getPose().heading.toDouble();

            // Rotación de vectores para Field Centric
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            x = rotX;
            y = rotY;
        }

        // Enviar potencias al RoadRunner
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(y, -x),
                -t
        ));
    }

    public void drive(double strafe, double forward, double turn) {
        drive(strafe, forward, turn, false);
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public void setSpeedMultiplier(double multiplier) {
        this.speedMultiplier = multiplier;
    }

    /**
     * Resetea el ángulo del robot a 0 (útil si el Field Centric se descalibra)
     */
    public void resetHeading() {
        // CORRECCIÓN: Obtenemos la pose actual del localizer
        Pose2d currentPose = drive.localizer.getPose();

        // Creamos una nueva pose con la misma posición X,Y pero ángulo 0
        Pose2d newPose = new Pose2d(currentPose.position.x, currentPose.position.y, 0);

        // CORRECCIÓN: Seteamos la pose en el localizer
        drive.localizer.setPose(newPose);
    }

    public Pose2d getPose() {
        // CORRECCIÓN: Acceso a través del localizer
        return drive.localizer.getPose();
    }

    public MecanumDrive getMecanumDrive() {
        return drive;
    }

    @Override
    public void periodic() {
        // Actualizar la odometría constantemente
        drive.updatePoseEstimate();
    }
}