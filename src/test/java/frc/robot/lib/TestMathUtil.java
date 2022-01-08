package frc.robot.lib;

import static frc.robot.lib.MathUtil.angleToBearing;
import static frc.robot.lib.MathUtil.getAngleDiff;
import static frc.robot.lib.MathUtil.normalise;
import static frc.robot.lib.MathUtil.normaliseBearing;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class TestMathUtil {
    @Test
    public void testGetAngleDiff() {
        assertEquals(getAngleDiff(0, 0), 0.0, 0.1);
        assertEquals(getAngleDiff(5, 0), 5.0, 0.1);
        assertEquals(getAngleDiff(0, 5), -5.0, 0.1);
        assertEquals(getAngleDiff(-5, 0), -5.0, 0.1);
        assertEquals(getAngleDiff(0, -5), 5.0, 0.1);
        // Weird angles that aren't in the range of -180...180
        assertEquals(getAngleDiff(361, 361), 0.0, 0.1);
        assertEquals(getAngleDiff(1, 361), 0.0, 0.1);
        assertEquals(getAngleDiff(-1, -361), 0.0, 0.1);
        assertEquals(getAngleDiff(361, -361), 2.0, 0.1);
        // Check the shortest distane calculation
        assertEquals(getAngleDiff(-175, 175), 10.0, 0.1);
        assertEquals(getAngleDiff(175, -175), -10.0, 0.1);
        // Directly opposite angles.
        assertEquals(getAngleDiff(270, 90), 180.0, 0.1);
        assertEquals(getAngleDiff(-90, 90), 180.0, 0.1);
        assertEquals(getAngleDiff(-270, -90), 180.0, 0.1);
    }

    @Test
    public void testNormalise() {
        assertEquals(normalise(10000, 10), 0, 0.1);
        assertEquals(normalise(15, 10), 5, 0.1);
        assertEquals(normalise(14.9, 10), 4.9, 0.1);
        assertEquals(normalise(10, 10), 0, 0.1);
        assertEquals(normalise(5, 10), 5, 0.1);
        assertEquals(normalise(0, 10), 0, 0.1);
        assertEquals(normalise(-5, 10), 5, 0.1);
        assertEquals(normalise(-10, 10), 0, 0.1);
        assertEquals(normalise(-14.9, 10), -4.9, 0.1);
        assertEquals(normalise(-15, 10), 5, 0.1);
        assertEquals(normalise(-10000, 10), 0, 0.1);
    }

    @Test
    public void testAngleToBearing() {
        assertEquals(angleToBearing(-180), 270, 0.1);
        assertEquals(angleToBearing(-90), 180, 0.1);
        assertEquals(angleToBearing(0), 90, 0.1);
        assertEquals(angleToBearing(90), 0, 0.1);
        assertEquals(angleToBearing(180), 270, 0.1);
        assertEquals(angleToBearing(360), 90, 0.1);
        assertEquals(angleToBearing(3600), 90, 0.1);
        assertEquals(angleToBearing(36000), 90, 0.1);
    }

    @Test
    public void testNormaliseBearing() {
        assertEquals(normaliseBearing(-360), 0, 0.1);
        assertEquals(normaliseBearing(-180), 180, 0.1);
        assertEquals(normaliseBearing(-10), 350, 0.1);
        assertEquals(normaliseBearing(0), 0, 0.1);
        assertEquals(normaliseBearing(10), 10, 0.1);
        assertEquals(normaliseBearing(90), 90, 0.1);
        assertEquals(normaliseBearing(180), 180, 0.1);
        assertEquals(normaliseBearing(350), 350, 0.1);
        assertEquals(normaliseBearing(360), 0, 0.1);
        assertEquals(normaliseBearing(3600), 0, 0.1);
    }
}
