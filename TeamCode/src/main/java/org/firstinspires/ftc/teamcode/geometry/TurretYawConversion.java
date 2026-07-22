package org.firstinspires.ftc.teamcode.geometry;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/**
 * Conversión pura ticks-de-encoder-del-motor ↔ yaw de torreta (convención de
 * {@code docs/plan-maestro/geometria-robot-mp04.md} sección 0: yaw CCW-positivo
 * visto desde arriba, cero = frente/intake). Implementa exactamente la fórmula ya
 * verificada por el equipo en esa hoja, sección 4.1 — no inventa una relación
 * nueva.
 *
 * Alcance deliberado: esto es sólo una conversión de unidades para lectura humana
 * (telemetría/diagnóstico). NO es el contrato formal de marcos de MP-04 (sección 4
 * de {@code 03-auto-aim-limelight-y-cancha.md}, que sigue pendiente de la prueba
 * física de cuatro anclas) y no calcula bearing a ningún goal — eso requiere el
 * marco Oficial/SDK, que esa tabla todavía no tiene confirmado. No usar esta clase
 * para nada que mueva la torreta; los soft limits reales siguen siendo
 * {@link TurretSubsystem#LIMIT_LEFT}/{@link TurretSubsystem#LIMIT_RIGHT} en ticks.
 */
public final class TurretYawConversion {

    /**
     * goBILDA 5203 751.8 PPR de salida (incluye reducción interna 26.9:1) por la
     * reducción externa de engranes 198T/68T (contrato-hardware.md, FND-003),
     * dividido entre 360°. Ver geometria-robot-mp04.md sección 4.1.
     */
    public static final double TICKS_PER_DEGREE = 751.8 * (198.0 / 68.0) / 360.0;

    private TurretYawConversion() {
    }

    /**
     * Signo invertido respecto a los ticks: el motor/cable está montado del lado
     * derecho del robot, así que ticks positivos barren hacia la derecha = yaw
     * NEGATIVO bajo la convención CCW-positivo (geometria-robot-mp04.md sección
     * 4.1, confirmado de memoria por el equipo con la posición física del
     * motor/cable como evidencia).
     */
    public static double ticksToYawDegrees(double ticks) {
        return -(ticks / TICKS_PER_DEGREE);
    }

    public static double yawDegreesToTicks(double yawDegrees) {
        return -(yawDegrees * TICKS_PER_DEGREE);
    }
}
