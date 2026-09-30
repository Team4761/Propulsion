package frc.team4761.propulsion.motors;

/**
 * Common surface for Team 4761's motor wrappers so subsystems can swap a Kraken for a Vortex
 * (or vice versa) without touching command code.
 *
 * <p>All angles and speeds are in <b>mechanism</b> units (after the gear ratio), unless the method
 * name says {@code Raw}, in which case they are motor-rotor units.
 */
public interface SmartMotor {
    /** Open-loop percent output, -1.0 (full reverse) to 1.0 (full forward). */
    void setSpeedPercent(double percent);

    /** Closed-loop velocity of the mechanism, in RPM. */
    void setSpeed(double mechanismRpm);

    /** Closed-loop velocity of the motor rotor, in RPM (ignores the gear ratio). */
    void setRawSpeed(double motorRpm);

    /**
     * Closed-loop move to an absolute mechanism angle.
     *
     * @return false (and does nothing) if the angle is outside the configured soft limits.
     */
    boolean set(double degrees);

    /**
     * Closed-loop move relative to the current mechanism angle.
     *
     * @return false (and does nothing) if the resulting angle is outside the soft limits.
     */
    boolean turn(double degrees);

    /** Current mechanism angle in degrees (continuous, not wrapped). */
    double getAngle();

    /** Current mechanism speed in RPM. */
    double getSpeedRPM();

    /** Re-zeros the encoder so the current position reads {@code degrees}. */
    void resetAngle(double degrees);

    /** True once the mechanism is within {@code toleranceDegrees} of {@code targetDegrees}. */
    boolean isAtAngle(double targetDegrees, double toleranceDegrees);

    /**
     * Stops driving the motor. If coasting is disabled the motor then actively holds its current
     * position; if coasting is enabled it is left at neutral output.
     */
    void stopTurning();

    /** After this, {@link #stopTurning()} releases the motor instead of holding position. */
    void enableCoasting();

    /** After this, {@link #stopTurning()} holds position. Also starts holding right away. */
    void disableCoasting();

    /** Same as {@link #getAngle()} but wrapped into [0, 360). Handy for WRAPPED mechanisms. */
    default double getWrappedAngle() {
        return MotorMath.wrapDegrees(getAngle());
    }
}
