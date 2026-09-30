package frc.team4761.propulsion.motors;

/**
 * Pure math used by the motor wrappers. Kept free of WPILib/vendor classes so it can be unit
 * tested on any machine.
 *
 * <p>Terminology: the "mechanism" is the thing you care about (turret, arm, roller). The "motor"
 * is the rotor. {@code gearRatio} is motor rotations per mechanism rotation, so a 10:1 reduction
 * has a gear ratio of 10.
 */
public final class MotorMath {
    private MotorMath() {}

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /** Clamps a duty cycle / percent output to [-1, 1]. */
    public static double clampDutyCycle(double output) {
        return clamp(output, -1.0, 1.0);
    }

    /** Wraps any angle into [0, 360). Unlike {@code %}, this never returns a negative angle. */
    public static double wrapDegrees(double degrees) {
        double wrapped = degrees % 360.0;
        return wrapped < 0.0 ? wrapped + 360.0 : wrapped;
    }

    /** Wraps any angle into [-180, 180). */
    public static double wrapDegreesSigned(double degrees) {
        double wrapped = wrapDegrees(degrees + 180.0) - 180.0;
        return wrapped;
    }

    /**
     * Returns the continuous target that reaches {@code targetDegrees} (interpreted modulo 360)
     * from {@code currentDegrees} by the shortest path.
     */
    public static double shortestPathTarget(double currentDegrees, double targetDegrees) {
        return currentDegrees + wrapDegreesSigned(targetDegrees - currentDegrees);
    }

    /**
     * Converts a requested mechanism angle into the continuous angle that should actually be
     * commanded, according to the motor mode.
     */
    public static double resolveTarget(MotorMode mode, double currentDegrees, double requestedDegrees) {
        return mode == MotorMode.WRAPPED
            ? shortestPathTarget(currentDegrees, requestedDegrees)
            : requestedDegrees;
    }

    /**
     * The angle that should be checked against the soft limits. In WRAPPED mode limits are
     * expressed in [0, 360).
     */
    public static double limitCheckAngle(MotorMode mode, double requestedDegrees) {
        return mode == MotorMode.WRAPPED ? wrapDegrees(requestedDegrees) : requestedDegrees;
    }

    public static boolean withinLimits(double degrees, double minDegrees, double maxDegrees) {
        return degrees >= minDegrees && degrees <= maxDegrees;
    }

    public static double mechanismDegreesToMotorRotations(double mechanismDegrees, double gearRatio) {
        return (mechanismDegrees / 360.0) * gearRatio;
    }

    public static double motorRotationsToMechanismDegrees(double motorRotations, double gearRatio) {
        return (motorRotations * 360.0) / gearRatio;
    }

    public static double mechanismRpmToMotorRpm(double mechanismRpm, double gearRatio) {
        return mechanismRpm * gearRatio;
    }

    public static double motorRpmToMechanismRpm(double motorRpm, double gearRatio) {
        return motorRpm / gearRatio;
    }

    /** True if {@code currentDegrees} is within {@code toleranceDegrees} of {@code targetDegrees}. */
    public static boolean isAtAngle(MotorMode mode, double currentDegrees, double targetDegrees, double toleranceDegrees) {
        double error = mode == MotorMode.WRAPPED
            ? wrapDegreesSigned(targetDegrees - currentDegrees)
            : targetDegrees - currentDegrees;
        return Math.abs(error) <= toleranceDegrees;
    }

    static void requirePositive(double value, String name) {
        if (!(value > 0.0) || !Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be a positive, finite number (got " + value + ")");
        }
    }
}
