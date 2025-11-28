package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LevnatadorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem2;


@Config
@Autonomous(name = "Autonomo One", group = "Autonomo uno")
public class AutonomoOne extends LinearOpMode {


    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(-52, 50, Math.toRadians(130));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        LevnatadorSubsystem levantar = new LevnatadorSubsystem(hardwareMap);
        ShooterSubsystem2 shooter = new ShooterSubsystem2(hardwareMap);


        Action adelante = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(-47, 40))
                .waitSeconds(3)
                .build();


        Action lado = drive.actionBuilder(new Pose2d(-47, 40, Math.toRadians(130)))
                .strafeTo(new Vector2d(-13, 25))
                .turn(Math.toRadians(-220))
                .lineToY(45)
                .waitSeconds(2)
                .splineToConstantHeading(new Vector2d(-47, 40), Math.toRadians(90))
                .turn(Math.toRadians(-150))
                .build();
        /*Action turn = drive.actionBuilder(new Pose2d(-13, 25, Math.toRadians(90)))
                .lineToY(45)
                .build();*/


        /*Action atras = drive.actionBuilder(new Pose2d(28, 45, Math.toRadians(180)))
                .strafeTo(new Vector2d(58, 45))
                .build();

        Action izquerda = drive.actionBuilder(new Pose2d(58, 45, Math.toRadians(180)))
                .strafeTo(new Vector2d(58, 15))
                .build();*/


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Listo para iniciar: presiona PLAY");
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(adelante);
            Actions.runBlocking(lado);

            if (isStopRequested()) return;
            telemetry.addLine("Trayectoria completada");
            telemetry.update();
        }
    }
}

