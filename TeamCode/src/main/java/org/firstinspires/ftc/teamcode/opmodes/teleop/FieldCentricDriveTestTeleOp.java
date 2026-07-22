package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drivetrain.FieldCentricDriveCommand;
import org.firstinspires.ftc.teamcode.opmodes.SafeCommandOpMode;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;

/** Field-centric drivetrain-only test. It maps no mechanism actuators. */
@TeleOp(name = "PRUEBA: FIELD CENTRIC CHASIS", group = "Test")
public final class FieldCentricDriveTestTeleOp extends SafeCommandOpMode {
    private PedroDriveSubsystem driveSubsystem;
    private GamepadEx driver;

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        driveSubsystem = new PedroDriveSubsystem(hardwareMap, 0, 0, 0, telemetry);

        driveSubsystem.setDefaultCommand(new FieldCentricDriveCommand(
                driveSubsystem,
                driver::getLeftX,
                driver::getLeftY,
                driver::getRightX));

        // Requiring the drivetrain interrupts its default command for one cycle,
        // guarantees zero output, and then makes the current front heading zero.
        new GamepadButton(driver, GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(() -> {
                    driveSubsystem.stop();
                    driveSubsystem.resetHeading();
                }, driveSubsystem));

        telemetry.addLine("=== PRUEBA FIELD CENTRIC ===");
        telemetry.addLine("Stick izq Y: forward/back");
        telemetry.addLine("Stick izq X: strafe left/right");
        telemetry.addLine("Stick der X: rotate left/right");
        telemetry.addLine("START: frente actual = heading 0");
        telemetry.addLine("BACK: E-STOP global");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("Heading field-centric", "%.1f deg",
                Math.toDegrees(driveSubsystem.getHeading()));
        telemetry.addData("Resets heading", driveSubsystem.getResetEpoch());
        telemetry.addData("Stick izq", "strafe %.2f | forward %.2f",
                driver.getLeftX(), driver.getLeftY());
        telemetry.addData("Stick der X", "rotate %.2f", driver.getRightX());
        telemetry.addLine("Puede combinar translacion y giro simultaneamente.");
        telemetry.update();
    }
}
