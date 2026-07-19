package org.firstinspires.ftc.teamcode.localization;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * DriveAdapter respaldado por el DriveSubsystem actual (Road Runner).
 * Trivial a propósito: existe para que MP-04/MP-05 se escriban contra la
 * interfaz y el cambio de ownership a Pedro (DEC-034) no toque consumidores.
 */
public class DriveSubsystemAdapter implements DriveAdapter {

    private final DriveSubsystem driveSubsystem;

    public DriveSubsystemAdapter(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        driveSubsystem.drive(strafeSpeed, forwardSpeed, turnSpeed);
    }

    @Override
    public void stop() {
        driveSubsystem.stop();
    }
}
