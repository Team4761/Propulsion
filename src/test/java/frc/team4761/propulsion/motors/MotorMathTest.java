package frc.team4761.propulsion.motors;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class MotorMathTest {
    private static final double EPS = 1e-9;

    @Test
    void wrapDegreesIsNeverNegative() {
        assertEquals(350.0, MotorMath.wrapDegrees(-10.0), EPS);
        assertEquals(10.0, MotorMath.wrapDegrees(370.0), EPS);
        assertEquals(0.0, MotorMath.wrapDegrees(720.0), EPS);
    }

    @Test
    void wrapDegreesSignedRange() {
        assertEquals(-10.0, MotorMath.wrapDegreesSigned(350.0), EPS);
        assertEquals(10.0, MotorMath.wrapDegreesSigned(-350.0), EPS);
        assertEquals(-180.0, MotorMath.wrapDegreesSigned(180.0), EPS);
    }

    @Test
    void wrappedModeTakesShortestPath() {
        // At 350, asking for 10 should go +20 (to 370), not -340.
        assertEquals(370.0, MotorMath.resolveTarget(MotorMode.WRAPPED, 350.0, 10.0), EPS);
        // At 10, asking for 350 should go -20.
        assertEquals(-10.0, MotorMath.resolveTarget(MotorMode.WRAPPED, 10.0, 350.0), EPS);
        // Works after many turns.
        assertEquals(725.0, MotorMath.resolveTarget(MotorMode.WRAPPED, 721.0, 5.0), EPS);
    }

    @Test
    void continuousModePassesThrough() {
        assertEquals(370.0, MotorMath.resolveTarget(MotorMode.CONTINUOUS, 0.0, 370.0), EPS);
        assertEquals(-45.0, MotorMath.limitCheckAngle(MotorMode.CONTINUOUS, -45.0), EPS);
    }

    @Test
    void wrappedLimitsUseWrappedAngle() {
        assertEquals(315.0, MotorMath.limitCheckAngle(MotorMode.WRAPPED, -45.0), EPS);
    }

    @Test
    void limitsAreInclusive() {
        assertTrue(MotorMath.withinLimits(-105.0, -105.0, 0.0));
        assertTrue(MotorMath.withinLimits(0.0, -105.0, 0.0));
        assertFalse(MotorMath.withinLimits(0.1, -105.0, 0.0));
    }

    @Test
    void gearRatioConversionsRoundTrip() {
        double ratio = 185.0 / 28.0;
        double rotations = MotorMath.mechanismDegreesToMotorRotations(90.0, ratio);
        assertEquals(ratio / 4.0, rotations, EPS);
        assertEquals(90.0, MotorMath.motorRotationsToMechanismDegrees(rotations, ratio), EPS);
        assertEquals(5000.0, MotorMath.motorRpmToMechanismRpm(MotorMath.mechanismRpmToMotorRpm(5000.0, 3.0), 3.0), EPS);
    }

    @Test
    void isAtAngleHandlesWrap() {
        assertTrue(MotorMath.isAtAngle(MotorMode.WRAPPED, 359.0, 1.0, 2.5));
        assertFalse(MotorMath.isAtAngle(MotorMode.CONTINUOUS, 359.0, 1.0, 2.5));
    }

    @Test
    void clampDutyCycle() {
        assertEquals(1.0, MotorMath.clampDutyCycle(3.25), EPS);
        assertEquals(-1.0, MotorMath.clampDutyCycle(-2.0), EPS);
    }

    @Test
    void rejectsBadGearRatio() {
        assertThrows(IllegalArgumentException.class, () -> MotorMath.requirePositive(0.0, "gearRatio"));
        assertThrows(IllegalArgumentException.class, () -> MotorMath.requirePositive(Double.NaN, "gearRatio"));
    }
}
