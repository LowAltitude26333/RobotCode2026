package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commands.ShooterPIDCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterHoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


@Config
@TeleOp(name = "Tuning: Dany", group = "Tuning")
public class ShooterTuningOpMode2 extends CommandOpMode {

    // --- VARIABLES EDITABLES EN DASHBOARD ---
    // Ajustado a tu máximo real aprox para evitar saturación del PID
    public static double TARGET_RPM = 3500;
    public static double TARGET_ANGLE = 0;

    private ShooterSubsystem shooter;
    private ShooterHoodSubsystem hood;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;

    private GamepadEx gamepad;

    private DcMotorEx shooterLeft, shooterRight;
    private VoltageSensor batteryVoltageSensor;
    private FtcDashboard dashboard;


    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        hood = new ShooterHoodSubsystem(hardwareMap, telemetry);
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterMotorDown");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterMotorUp");



        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        dashboard = FtcDashboard.getInstance();


        gamepad = new GamepadEx(gamepad1);

        // Default Command: Hood obedece al Dashboard
        hood.setDefaultCommand(new RunCommand(() -> hood.setAngle(TARGET_ANGLE), hood));

        // --- CONTROLES ---

        // 1. SHOOTER PID (Y -> On | A -> Off)
        new GamepadButton(gamepad, GamepadKeys.Button.Y)
                .whenPressed(new ShooterPIDCommand(shooter, TARGET_RPM));

        new GamepadButton(gamepad, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(shooter::stop, shooter));

        // 2. SHOOTER MANUAL / MAX POWER (D-Pad Right)
        // CORRECCIÓN: Usamos setTargetRPM con el máximo real en lugar de driveShooter(1.0)
        // para no pelear con el periodic() del subsistema.
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> shooter.setTargetRPM(3500), shooter));

        // 3. INTAKE
        new GamepadButton(gamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(intake::intakeOn, intake));
        new GamepadButton(gamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(intake::intakeReturn, intake));
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(intake::intakeOff, intake));

        // 4. KICKER
        new GamepadButton(gamepad, GamepadKeys.Button.X)
                .whileHeld(new RunCommand(kicker::kick, kicker))
                .whenReleased(new InstantCommand(kicker::stop, kicker));
        new GamepadButton(gamepad, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(kicker::stop, kicker));
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(kicker::reverse, kicker));

        telemetry.addLine("--- LISTO PARA TUNING ---");
        telemetry.update();
    }

    @Override
    public void run() {
        // TRUCO DE TUNING EN VIVO
        // Actualiza el target del subsistema si cambias la variable en Dashboard
        // y el motor ya está corriendo (target != 0)
        if (shooter.getTargetRPM() != 0 && shooter.getTargetRPM() != TARGET_RPM) {
            shooter.setTargetRPM(TARGET_RPM);
        }

        super.run();

        telemetry.addData(">> DASHBOARD <<", "Edita TARGET_RPM");
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Actual RPM", shooter.getShooterRPM());
        telemetry.update();
        // --- EXTRA: Voltaje y Corriente al Dashboard ---
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Battery Voltage (V)", batteryVoltageSensor.getVoltage());
        packet.put("Shooter Left Current (A)", shooterLeft.getCurrent(CurrentUnit.AMPS));
        packet.put("Shooter Right Current (A)", shooterRight.getCurrent(CurrentUnit.AMPS));
        packet.put("Shooter Left RPM", shooterLeft.getVelocity() * 60.0 / ShooterSubsystem.TICKS_PER_REV);
        packet.put("Shooter Right RPM", shooterRight.getVelocity() * 60.0 / ShooterSubsystem.TICKS_PER_REV);
        dashboard.sendTelemetryPacket(packet);


    }
}