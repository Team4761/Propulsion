package frc.team4761.propulsion.motors;

/** How a {@link SmartMotor} interprets angle targets. */
public enum MotorMode {
    /**
     * Angles are absolute and unbounded (e.g. an arm or intake pivot). Asking for 370 degrees
     * means one full turn past 10 degrees.
     */
    CONTINUOUS,
    /**
     * Angles wrap every 360 degrees (e.g. a turret with a slip ring or anything that spins freely).
     * The motor always takes the shortest path to the requested angle.
     */
    WRAPPED
}
