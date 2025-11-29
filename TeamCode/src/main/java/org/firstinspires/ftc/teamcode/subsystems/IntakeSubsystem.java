package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotMap;

public class IntakeSubsystem extends SubsystemBase {
    private final MotorEx intakeMotor;

    public IntakeSubsystem(HardwareMap hardwareMap)   {
        intakeMotor = new MotorEx(hardwareMap, RobotMap.INTAKE_MOTOR);

        intakeMotor.setInverted(RobotMap.INTAKE_MOTOR_IS_INVERTED);
    }

    public void intakeOn (){
        intakeMotor.set(LowAltitudeConstants.INTAKE_IN_SPEED);
    }

    public void intakeOff () {
        intakeMotor.set(LowAltitudeConstants.INTAKE_STOP);
    }
    public void intakeReturn(){
        intakeMotor.set(LowAltitudeConstants.INTAKE_REVERSE);
    }
    public Action soltar() {
        return (telemetryPacket) -> {

            intakeMotor.set(-0.7);

            return false;
        };
    }
    public Action off() {
        return (telemetryPacket) -> {

            intakeMotor.set(0.0);

            return false;
        };
    }
}