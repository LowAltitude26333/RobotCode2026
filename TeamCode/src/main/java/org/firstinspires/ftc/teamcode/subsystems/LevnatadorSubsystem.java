package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotMap;

public class LevnatadorSubsystem extends SubsystemBase {
    private final MotorEx levantador;

    public LevnatadorSubsystem(HardwareMap hardwareMap) {

        levantador= new MotorEx(hardwareMap, RobotMap.LEVANTADOR);

    }

    public void left2() {

        levantador.set((0.7));

    }

    public void reverse(){
        levantador.set(-0.7);
    }



    public void stop() {

        levantador.set(0.0);

    }
    public Action cargar() {
        return (telemetryPacket) -> {

            levantador.set(0.7);

            return false;
        };
    }
}
