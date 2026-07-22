package org.firstinspires.ftc.teamcode.shooter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.List;
import org.junit.Test;

public class ShotDatasetCsvTest {

    @Test
    public void parsesSamplesIgnoringBlankAndCommentLines() {
        String csv = "# distanceInches,rpm\n"
                + "\n"
                + "24,2450\n"
                + "   \n"
                + "36,2900\n"
                + "# trailing comment\n";

        List<ShotSample> samples = ShotDatasetCsv.parseSamples(csv);

        assertEquals(2, samples.size());
        assertEquals(24.0, samples.get(0).distanceInches, 0.0);
        assertEquals(2450.0, samples.get(0).rpm, 0.0);
        assertEquals(36.0, samples.get(1).distanceInches, 0.0);
        assertEquals(2900.0, samples.get(1).rpm, 0.0);
    }

    @Test
    public void malformedSampleRowThrowsWithLineNumber() {
        String csv = "24,2450\nnot-a-number,2900\n";
        try {
            ShotDatasetCsv.parseSamples(csv);
            fail("expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("línea 2"));
        }
    }

    @Test
    public void wrongFieldCountThrowsWithLineNumber() {
        String csv = "24,2450,extra\n";
        try {
            ShotDatasetCsv.parseSamples(csv);
            fail("expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("línea 1"));
        }
    }

    @Test
    public void parsesTrialsWithCaseInsensitiveEnums() {
        String csv = "# sessionId,distanceGroupId,modelKind,role,distanceInches,targetRpm,"
                + "measuredRpmAtFeed,rpmReadyHoldMs,outcome\n"
                + "2026-07-25-AM,D24,linear,calibration,24,2450,2455,300,scored\n"
                + "2026-07-25-AM,D36,LINEAR,HOLDOUT,36,2900,2810,260,SHORT\n";

        List<ShotTrial> trials = ShotDatasetCsv.parseTrials(csv);

        assertEquals(2, trials.size());
        ShotTrial first = trials.get(0);
        assertEquals("2026-07-25-AM", first.sessionId);
        assertEquals("D24", first.distanceGroupId);
        assertEquals(RpmModelKind.LINEAR, first.modelKind);
        assertEquals(ShotDatasetRole.CALIBRATION, first.role);
        assertEquals(24.0, first.distanceInches, 0.0);
        assertEquals(2450.0, first.targetRpm, 0.0);
        assertEquals(2455.0, first.measuredRpmAtFeed, 0.0);
        assertEquals(300L, first.rpmReadyHoldMs);
        assertEquals(ShotOutcome.SCORED, first.outcome);
        assertTrue(first.scored());

        ShotTrial second = trials.get(1);
        assertEquals(ShotDatasetRole.HOLDOUT, second.role);
        assertEquals(ShotOutcome.SHORT, second.outcome);
        assertTrue(!second.scored());
    }

    @Test
    public void invalidEnumValueThrowsWithLineNumber() {
        String csv = "s1,D24,LINEAR,calibration,24,2450,2455,300,scored\n"
                + "s1,D36,NOT_A_KIND,holdout,36,2900,2810,260,short\n";
        try {
            ShotDatasetCsv.parseTrials(csv);
            fail("expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("línea 2"));
        }
    }

    @Test
    public void nullCsvReturnsEmptyList() {
        assertTrue(ShotDatasetCsv.parseSamples(null).isEmpty());
        assertTrue(ShotDatasetCsv.parseTrials(null).isEmpty());
    }
}
