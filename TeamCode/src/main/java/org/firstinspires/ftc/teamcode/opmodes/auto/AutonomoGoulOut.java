package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;


@Config
@Autonomous(name = " AutonomoGoulOut ", group = " AutonomoGoulOut ")
public class AutonomoGoulOut extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(-50, 48, Math.toRadians(130));//pegado a la goul frente
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap,telemetry);
        KickerSubsystem kicker = new KickerSubsystem(hardwareMap);
        /*
        ===========================
        Thread para mostrar RPM TnT
        ===========================
         */
        Thread rpmThread = new Thread(() -> {
            try {
                while (!isStopRequested()) {
                    if (opModeIsActive()) {
                        telemetry.addData("ShooterRPM", shooter.getShooterRPM());
                    } else {
                        telemetry.addData("ShooterRPM", "init");
                    }
                    telemetry.update();
                    Thread.sleep(100);
                }
            } catch (InterruptedException ignored) { }
        });
        rpmThread.setDaemon(true); // para que no impida que el programa termine
        rpmThread.start();

        Thread mostrarRPMThread = new Thread(() -> {
            try {
                while (!isStopRequested()) {
                    telemetry.addData("ShooterRPM", shooter.getShooterRPM());
                    telemetry.update();
                    Thread.sleep(80); // Frecuencia de actualización
                }
            } catch (InterruptedException ignored) {}
        });
        mostrarRPMThread.setDaemon(true);



        Action uno = drive.actionBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-15,63,Math.toRadians(180)),Math.toRadians(130))
                .build();




        Action mostrarRPM = (tp) -> {
            telemetry.addData("ShooterRPM", shooter.getShooterRPM());
            telemetry.update();
            return false;
        };

        /*Action unoConIntake = new SequentialAction(
                new ParallelAction(
                        dos,intake.soltar()
                ),
                intake.off() // se apaga después
        );
*/

        Action unoConTodo = new SequentialAction(

                shooter.setRPMAutonomous(3000),

                new ParallelAction(
                        // termina si llega a RPM
                        shooter.waitUntilTargetRPMAutonomous(),

                        /* o si pasan 5 segundos
                        new SleepAction(5.0),

                         */

                        // pero mientras tanto corre PID
                        shooter.runPIDAutonomous(),
                        mostrarRPM
                ),
                mostrarRPM,
                uno
        );
        /*
        Action active = new SequentialAction(
                new ParallelAction(
                        kicker.cargar(),
                        intake.soltar()

                ),
                new SleepAction(3.0),
                intake.off(),
                kicker.off()

        );

         */





        /*Action cinco = drive.actionBuilder(new Pose2d(13, 30, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(13,30,Math.toRadians(270)),Math.toRadians(130))
                .build();*/


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Listo para iniciar: presiona PLAY");
            telemetry.update();
        }

        waitForStart();

        mostrarRPMThread.start();

        if (opModeIsActive()) {
            Actions.runBlocking(uno);





            if (isStopRequested()) return;
            telemetry.addLine("Trayectoria completada");
            telemetry.update();
        }


    }
}