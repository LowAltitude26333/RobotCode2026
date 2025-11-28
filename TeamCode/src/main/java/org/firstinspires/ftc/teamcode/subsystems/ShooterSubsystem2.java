package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotMap;

public class ShooterSubsystem2 extends SubsystemBase {
    private final MotorEx shooter2;
    private final MotorEx shooter3;

    public ShooterSubsystem2(HardwareMap hardwareMap) {

        shooter2 = new MotorEx(hardwareMap, RobotMap.SHOOTER2);
        shooter3 = new MotorEx(hardwareMap, RobotMap.SHOOTER3);

    }

    public void right2() {

        shooter2.set(0.6);
        shooter3.set(0.6);

    }

    public void left2() {

        shooter2.set((-0.7));
        shooter3.set(-0.7);

    }

    public void stop() {

        shooter2.set(0.0);
        shooter3.set(0);

    }

    public Action disparar() {
        return (telemetryPacket) -> {

            shooter2.set(-0.6);
            shooter3.set(-0.6);

            return false;
        };
    }
}