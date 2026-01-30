package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.ColorDetectCommand;
import org.firstinspires.ftc.teamcode.commands.PoseStorage;
import org.firstinspires.ftc.teamcode.commands.ShooterPIDCommand;
import org.firstinspires.ftc.teamcode.commands.auto.ShootBurstCommand;
import org.firstinspires.ftc.teamcode.subsystems.ColorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterHoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous(name = "Beta")
public class AutonomoBetaPosition extends CommandOpMode {

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private ShooterHoodSubsystem hood;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;

    @Override
    public void initialize() {
        Pose2d startPose = new Pose2d(-52, 47, Math.toRadians(127));

        drive = new DriveSubsystem(hardwareMap, startPose, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        hood = new ShooterHoodSubsystem(hardwareMap, telemetry);
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        MecanumDrive rrDrive = drive.getMecanumDrive();

        Action path1 = rrDrive.actionBuilder(startPose)
                .strafeTo(new Vector2d(-30, 25))
                .build();

        Action path2 = rrDrive.actionBuilder(new Pose2d(-30, 25, Math.toRadians(127)))
                .splineToLinearHeading(new Pose2d(-10,20,Math.toRadians(270)),Math.toRadians(130))
                .strafeTo(new Vector2d(-10, 50))
                .build();

        Action path3 = rrDrive.actionBuilder(new Pose2d(-10, 50, Math.toRadians(270)))
                .strafeTo(new Vector2d(-10, 45))
                .splineToLinearHeading(new Pose2d(-30,25,Math.toRadians(130)),Math.toRadians(270))
                .build();

        Action path4 = rrDrive.actionBuilder(new Pose2d(-30, 25, Math.toRadians(130)))
                .splineToLinearHeading(new Pose2d(10,25,Math.toRadians(270)),Math.toRadians(130))
                .strafeTo(new Vector2d(10, 50))
                .build();

        Action path5 = rrDrive.actionBuilder(new Pose2d(10, 50, Math.toRadians(270)))
                .strafeTo(new Vector2d(10, 45))
                .splineToLinearHeading(new Pose2d(-30,25,Math.toRadians(127)),Math.toRadians(270))
                //.splineToLinearHeading(new Pose2d(-30,25,Math.toRadians(90)),Math.toRadians(127))
                .build();

        schedule(new ParallelCommandGroup(
                new ShooterPIDCommand(shooter, 2550),
                new InstantCommand(intake::intakeOn, intake),

                new SequentialCommandGroup(
                        new InstantCommand(() -> hood.setPosition(LowAltitudeConstants.HoodPosition.SHORT_SHOT), hood),
                        new ActionCommand(path1, drive),
                        new ShootBurstCommand(shooter,hood, kicker, 3),
                        new ActionCommand(path2, drive),
                        new InstantCommand(kicker::kick, kicker),
                        new WaitCommand(150),
                        new InstantCommand(kicker::stop, kicker),
                        new WaitCommand(500),
                        new ActionCommand(path3, drive),
                        new ShootBurstCommand(shooter,hood, kicker, 3),
                        new ActionCommand(path4, drive),
                        new InstantCommand(kicker::kick, kicker),
                        new WaitCommand(150),
                        new InstantCommand(kicker::stop, kicker),
                        new WaitCommand(500),
                        new ActionCommand(path5, drive),
                        new ShootBurstCommand(shooter,hood, kicker, 3),
                        new InstantCommand(shooter::stop),
                        new InstantCommand(intake::intakeOff),

                        // 🔹 Guardar pose final para TeleOp
                        new InstantCommand(() -> PoseStorage.currentPose = rrDrive.pose)
                )
        ));

        telemetry.addLine("Auto Beta cargado. Pose final se guardará en PoseStorage.");
        telemetry.update();
    }
}
