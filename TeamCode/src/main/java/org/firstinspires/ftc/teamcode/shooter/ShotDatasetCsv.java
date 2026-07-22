package org.firstinspires.ftc.teamcode.shooter;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Convierte el formato de fila que Tuning puede llenar en una hoja de cálculo
 * (Paso 5 de {@code plan-paralelo-20h.md}) en {@code List<ShotSample>}/
 * {@code List<ShotTrial>} para {@link RpmModelSelector}, en vez de escribir cada
 * punto a mano como código Java. No lee archivos ni depende de Android — recibe
 * el contenido ya como texto para poder probarse como JVM puro y para que el
 * caller decida el origen (asset, archivo local, texto pegado).
 *
 * Formato: una fila por línea, campos separados por coma. Líneas vacías o que
 * empiezan con {@code #} se ignoran (incluida una fila de encabezado, si se
 * quiere conservar una en el archivo original — coméntala con {@code #}). Una
 * fila mal formada lanza {@link IllegalArgumentException} con el número de
 * línea; no se descarta en silencio.
 */
public final class ShotDatasetCsv {

    private ShotDatasetCsv() {
    }

    /** Filas: {@code distanceInches,rpm} — ver {@link ShotSample}. */
    public static List<ShotSample> parseSamples(String csv) {
        List<ShotSample> samples = new ArrayList<>();
        for (Line line : usableLines(csv)) {
            String[] fields = splitExact(line, 2);
            try {
                samples.add(new ShotSample(
                        Double.parseDouble(fields[0].trim()),
                        Double.parseDouble(fields[1].trim())));
            } catch (IllegalArgumentException e) {
                throw invalidLine("ShotSample", line, e);
            }
        }
        return samples;
    }

    /**
     * Filas: {@code sessionId,distanceGroupId,modelKind,role,distanceInches,
     * targetRpm,measuredRpmAtFeed,rpmReadyHoldMs,outcome} — ver {@link ShotTrial}.
     * {@code modelKind}/{@code role}/{@code outcome} no distinguen mayúsculas.
     */
    public static List<ShotTrial> parseTrials(String csv) {
        List<ShotTrial> trials = new ArrayList<>();
        for (Line line : usableLines(csv)) {
            String[] fields = splitExact(line, 9);
            try {
                trials.add(new ShotTrial(
                        fields[0].trim(),
                        fields[1].trim(),
                        RpmModelKind.valueOf(upper(fields[2])),
                        ShotDatasetRole.valueOf(upper(fields[3])),
                        Double.parseDouble(fields[4].trim()),
                        Double.parseDouble(fields[5].trim()),
                        Double.parseDouble(fields[6].trim()),
                        Long.parseLong(fields[7].trim()),
                        ShotOutcome.valueOf(upper(fields[8]))));
            } catch (IllegalArgumentException e) {
                throw invalidLine("ShotTrial", line, e);
            }
        }
        return trials;
    }

    private static String upper(String field) {
        return field.trim().toUpperCase(Locale.ROOT);
    }

    private static IllegalArgumentException invalidLine(String kind, Line line, Exception cause) {
        return new IllegalArgumentException(
                kind + " inválido en línea " + line.number + ": \"" + line.text + "\"", cause);
    }

    private static String[] splitExact(Line line, int expectedFields) {
        String[] fields = line.text.split(",", -1);
        if (fields.length != expectedFields) {
            throw new IllegalArgumentException("línea " + line.number + " tiene " + fields.length
                    + " campo(s), se esperaban " + expectedFields + ": \"" + line.text + "\"");
        }
        return fields;
    }

    private static List<Line> usableLines(String csv) {
        List<Line> lines = new ArrayList<>();
        if (csv == null) {
            return lines;
        }
        String[] rawLines = csv.split("\n", -1);
        for (int i = 0; i < rawLines.length; i++) {
            String trimmed = rawLines[i].trim();
            if (trimmed.isEmpty() || trimmed.startsWith("#")) {
                continue;
            }
            lines.add(new Line(i + 1, trimmed));
        }
        return lines;
    }

    private static final class Line {
        final int number;
        final String text;

        Line(int number, String text) {
            this.number = number;
            this.text = text;
        }
    }
}
