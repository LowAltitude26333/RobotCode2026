package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp(name = "TeleOp Intake Control")
public class IntakeTeleOp extends CommandOpMode {

    private IntakeSubsystem intake;
    private GamepadEx driverGamepad;

    @Override
    public void initialize() {
        // Inicializamos el subsistema y el mando
        intake = new IntakeSubsystem(hardwareMap);
        driverGamepad = new GamepadEx(gamepad1);

        // --- ASIGNACIÓN DE BOTONES ---

        // Bumper derecho: Prender (Intake On)
        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(intake::intakeOn, intake));

        // Bumper izquierdo: Reversa (Intake Return)
        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(intake::intakeReturn, intake));

        // D-pad Down: Apagar (Intake Off)
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(intake::intakeOff, intake));
    }
}