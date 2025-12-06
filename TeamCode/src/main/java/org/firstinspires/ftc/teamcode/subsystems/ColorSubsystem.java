package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class ColorSubsystem extends SubsystemBase {

    private final RevColorSensorV3 colorSensor;
    private final Telemetry telemetry;

    public ColorSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
        this.telemetry = telemetry;
    }

    public String getColor() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        String detected = "NADA";
        if (g > r && g > b) detected = "VERDE";
        if (r > g && b > g) detected = "MORADO";

        telemetry.addData("Rojo", r);
        telemetry.addData("Verde", g);
        telemetry.addData("Azul", b);
        telemetry.addData("Color detectado", detected);
        telemetry.update();

        return detected;
    }
}