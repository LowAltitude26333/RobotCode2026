package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;


@Config
@Autonomous(name = "Autonomo One", group = "Autonomo Full")
public class AutonomoFullCore extends LinearOpMode {


    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(-52, 50, Math.toRadians(130));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap,telemetry);
        KickerSubsystem kicker = new KickerSubsystem(hardwareMap);


        Action uno = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(-47, 40))
                .waitSeconds(3)
                .build();


        Action dos = drive.actionBuilder(new Pose2d(-47, 40, Math.toRadians(130)))
                .splineToLinearHeading(new Pose2d(-13,25,Math.toRadians(270)),Math.toRadians(130))
                .strafeTo(new Vector2d(-13, 45))
                .waitSeconds(0.2)
                .build();
        Action tres = drive.actionBuilder(new Pose2d(-13, 45, Math.toRadians(270)))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-47,40,Math.toRadians(130)),Math.toRadians(270))
                .waitSeconds(0.2)
                .build();


        Action cuatro = drive.actionBuilder(new Pose2d(-47, 40, Math.toRadians(130)))
                .splineToLinearHeading(new Pose2d(13,30,Math.toRadians(270)),Math.toRadians(130))
                .build();

        Action unoConIntake = new SequentialAction(
                new ParallelAction(
                        dos,intake.soltar()
                ),
                intake.off() // se apaga después
        );
        /*Action unoConTodo = new SequentialAction(
                new ParallelAction(
                        uno,
                        shooter.disparar()
                ));*/
        Action active = new SequentialAction(
                new ParallelAction(
                        kicker.cargar(),
                        intake.soltar()

                ),
                new SleepAction(3.0),
                intake.off(),
                kicker.off()

        );




        /*Action cinco = drive.actionBuilder(new Pose2d(13, 30, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(13,30,Math.toRadians(270)),Math.toRadians(130))
                .build();*/


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Listo para iniciar: presiona PLAY");
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive()) {
           // Actions.runBlocking(unoConTodo);
            Actions.runBlocking(active);
            Actions.runBlocking(unoConIntake);
            Actions.runBlocking(tres);
            Actions.runBlocking(cuatro);




            if (isStopRequested()) return;
            telemetry.addLine("Trayectoria completada");
            telemetry.update();
        }
    }
}

