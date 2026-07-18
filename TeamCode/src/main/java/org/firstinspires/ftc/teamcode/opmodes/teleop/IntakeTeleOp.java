package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.opmodes.SafeCommandOpMode;

@TeleOp(name = "TeleOp Intake Control")
@Disabled
public class IntakeTeleOp extends SafeCommandOpMode {

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
                .whileHeld(new RunCommand(intake::intakeOn, intake))
                .whenReleased(new InstantCommand(intake::intakeOff, intake));

        // Bumper izquierdo: Reversa (Intake Return)
        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new RunCommand(intake::intakeReturn, intake))
                .whenReleased(new InstantCommand(intake::intakeOff, intake));

        // D-pad Down: Apagar (Intake Off)
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(intake::intakeOff, intake));



    }
}
