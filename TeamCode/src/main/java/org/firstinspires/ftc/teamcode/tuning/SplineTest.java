package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            try {
                waitForStart();
                if (isStopRequested()) return;

                Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
            } finally {
                drive.leftFront.setPower(0);
                drive.leftBack.setPower(0);
                drive.rightBack.setPower(0);
                drive.rightFront.setPower(0);
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);
            try {
                waitForStart();
                if (isStopRequested()) return;

                Actions.runBlocking(
                        drive.actionBuilder(beginPose)
                                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                                .splineTo(new Vector2d(0, 60), Math.PI)
                                .build());
            } finally {
                drive.leftMotors.forEach(motor -> motor.setPower(0));
                drive.rightMotors.forEach(motor -> motor.setPower(0));
            }
        } else {
            throw new RuntimeException();
        }
    }
}
