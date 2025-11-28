package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class GoToPoseCommandAction extends CommandBase {

    private final MecanumDrive drive;
    private final Pose2d targetPose;
    private Action action;

    public GoToPoseCommandAction(MecanumDrive drive, Pose2d targetPose) {
        this.drive = drive;
        this.targetPose = new Pose2d(-52, 50, Math.toRadians(130));


    }

    @Override
    public void initialize() {

        Pose2d currentPose = drive.localizer.getPose();

        action = drive.actionBuilder(currentPose)
                .strafeTo(targetPose.position)
                .turn(targetPose.heading.toDouble() - currentPose.heading.toDouble())
                .build();
        Actions.runBlocking(action);
        }


        @Override
        public boolean isFinished() {
            return true; // El command termina al finalizar la acción
    }
}
