package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry; // Importante
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotMap;

public class DriveSubsystem extends SubsystemBase {

    private final MecanumDrive drive;
    private final com.arcrobotics.ftclib.drivebase.MecanumDrive driveAuto;
    private double speedMultiplier = 1.0;
    private Telemetry telemetry;

    private final MotorEx frontLeft, frontRight, backLeft, backRight;

    // Agregamos Telemetry al constructor para debug
    public DriveSubsystem(HardwareMap hardwareMap, Pose2d startPose, Telemetry telemetry) {
        frontLeft = new MotorEx(hardwareMap, RobotMap.FRONT_LEFT_MOTOR);
        frontRight = new MotorEx(hardwareMap, RobotMap.FRONT_RIGHT_MOTOR);
        backLeft = new MotorEx(hardwareMap, RobotMap.BACK_LEFT_MOTOR);
        backRight = new MotorEx(hardwareMap, RobotMap.BACK_RIGHT_MOTOR);

        backLeft.setInverted(RobotMap.BACK_LEFT_MOTOR_IS_INVERTED);
        frontLeft.setInverted(RobotMap.FRONT_LEFT_MOTOR_IS_INVERTED);
        backRight.setInverted(RobotMap.BACK_RIGHT_MOTOR_IS_INVERTED);
        frontRight.setInverted(RobotMap.FRONT_RIGHT_MOTOR_IS_INVERTED);

        frontLeft.encoder.reset();
        frontRight.encoder.reset();
        backLeft.encoder.reset();
        backRight.encoder.reset();

        this.drive = new MecanumDrive(hardwareMap, startPose);
        this.driveAuto = new com.arcrobotics.ftclib.drivebase.MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        this.telemetry = telemetry;
    }

    public void drive(double strafe, double forward, double turn, boolean isFieldCentric) {

        // Debug: Ver qué valores llegan realmente
        telemetry.addData("DRIVE Input Strafe", strafe);
        telemetry.addData("DRIVE Input Fwd", forward);
        telemetry.addData("DRIVE Input Turn", turn);

        double x = strafe * speedMultiplier;
        double y = forward * speedMultiplier;
        double t = turn * speedMultiplier;

        if (isFieldCentric) {
            double heading = drive.localizer.getPose().heading.toDouble();
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
            x = rotX;
            y = rotY;
        }

        // IMPORTANTE: El 'turn' en RoadRunner 1.0 a veces requiere un valor más alto
        // si maxAngVel es muy alto. Pero probemos directo primero.
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(y, -x),
                -t
        ));
    }

    public void drive(double strafe, double forward, double turn) {
        drive(strafe, forward, turn, false);
    }

    public void resetHeading() {
        Pose2d currentPose = drive.localizer.getPose();
        drive.localizer.setPose(new Pose2d(currentPose.position.x, currentPose.position.y, 0));
    }

    public MecanumDrive getMecanumDrive() {
        return drive;
    }

    public double getPoseX(){
        return drive.localizer.getPose().position.x;

    }
    public double getPoseY(){
        return drive.localizer.getPose().position.y;

    }
    public double getHeading(){
        return drive.localizer.getPose().heading.toDouble();

    }

    @Override
    public void periodic() {
        drive.updatePoseEstimate();
        // Telemetría de posición
        Pose2d p = drive.localizer.getPose();
        telemetry.addData("ROBOT X", p.position.x);
        telemetry.addData("ROBOT Y", p.position.y);
        telemetry.addData("ROBOT Heading (Deg)", Math.toDegrees(p.heading.toDouble()));
    }

}