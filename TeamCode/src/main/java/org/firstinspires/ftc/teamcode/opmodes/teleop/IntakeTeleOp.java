package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.opmodes.SafeCommandOpMode;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;

@TeleOp(name = "TeleOp Intake Control")
public class IntakeTeleOp extends SafeCommandOpMode {

    private IntakeSubsystem intake;
    private KickerSubsystem kicker;
    private GamepadEx driverGamepad;

    @Override
    public void initialize() {
        // Inicializamos el subsistema y el mando
        intake = new IntakeSubsystem(hardwareMap);
        kicker = new KickerSubsystem(hardwareMap);
        driverGamepad = new GamepadEx(gamepad1);

        // --- ASIGNACIÓN DE BOTONES ---

        // Bumper derecho: Prender (Intake On)
        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(intake::intakeOn, intake));

        // Bumper izquierdo: Reversa (Intake Return)
        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(intake::intakeReturn, intake));

        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                        .whenPressed(new InstantCommand(kicker::kick, kicker));

        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(kicker::reverse, kicker));

        // D-pad Down: Apagar (Intake Off)
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(intake::intakeOff, intake))
                .whenPressed(new InstantCommand(kicker::stop, kicker));

    }
}
