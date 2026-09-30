package frc.team4761.propulsion.motors;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * A NEO Vortex on a SPARK Flex with mechanism-unit helpers. Same API as {@link SmartKrakenMotor}.
 *
 * <pre>{@code
 * SmartVortexMotor spindexer = SmartVortexMotor.Builder.newInstance()
 *     .canId(43)
 *     .velocityPID(0.0002, 0.0, 0.0)
 *     .velocityFF(0.1, 0.0017)     // volts, volts per motor RPM
 *     .gearRatio(5.0)
 *     .smartCurrentLimit(40)
 *     .build();
 * }</pre>
 *
 * <p>Position control uses closed-loop slot 0, velocity control uses slot 1. If you never call
 * {@link Builder#velocityPID}, the position gains are copied into slot 1.
 *
 * <p>The SPARK's encoder is left in native units (rotations, RPM); this class does the gear ratio
 * math itself so both wrappers behave identically.
 */
public class SmartVortexMotor implements SmartMotor {
    private final SparkFlex motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;
    private final double gearRatio;
    private final boolean hasAngleLimits;
    private final double minAngle;
    private final double maxAngle;
    private final MotorMode mode;
    private final String name;
    private boolean coastingEnabled = true;

    public SmartVortexMotor(Builder builder) {
        MotorMath.requirePositive(builder.gearRatio, "gearRatio");
        this.motor = new SparkFlex(builder.canId, MotorType.kBrushless);
        this.encoder = this.motor.getEncoder();
        this.controller = this.motor.getClosedLoopController();
        this.gearRatio = builder.gearRatio;
        this.hasAngleLimits = builder.hasAngleLimits;
        this.minAngle = builder.minAngle;
        this.maxAngle = builder.maxAngle;
        this.mode = builder.mode;
        this.name = "SmartVortexMotor[" + builder.canId + "]";

        SparkFlexConfig config = new SparkFlexConfig();
        config.inverted(builder.inverted)
            .idleMode(builder.brakeMode ? IdleMode.kBrake : IdleMode.kCoast)
            .smartCurrentLimit(builder.smartCurrentLimit);
        config.closedLoop
            .pid(builder.p, builder.i, builder.d, ClosedLoopSlot.kSlot0)
            .pid(
                builder.velocityGainsSet ? builder.velocityP : builder.p,
                builder.velocityGainsSet ? builder.velocityI : builder.i,
                builder.velocityGainsSet ? builder.velocityD : builder.d,
                ClosedLoopSlot.kSlot1
            )
            .outputRange(builder.minOutput, builder.maxOutput, ClosedLoopSlot.kSlot0)
            .outputRange(builder.minOutput, builder.maxOutput, ClosedLoopSlot.kSlot1);
        config.closedLoop.feedForward
            .kS(builder.kS, ClosedLoopSlot.kSlot1)
            .kV(builder.kV, ClosedLoopSlot.kSlot1);

        REVLibError status = this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        if (status != REVLibError.kOk) {
            DriverStation.reportError(name + " failed to apply config: " + status, false);
        }

        this.encoder.setPosition(MotorMath.mechanismDegreesToMotorRotations(builder.startingAngle, gearRatio));
    }

    @Override
    public void setSpeedPercent(double percent) {
        this.motor.set(MotorMath.clampDutyCycle(percent));
    }

    /** Kept for compatibility with 2026 code; same as {@link #setSpeedPercent}. */
    public void setRawSpeedPercent(double percent) {
        setSpeedPercent(percent);
    }

    @Override
    public void setRawSpeed(double motorRpm) {
        this.controller.setSetpoint(motorRpm, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }

    @Override
    public void setSpeed(double mechanismRpm) {
        setRawSpeed(MotorMath.mechanismRpmToMotorRpm(mechanismRpm, this.gearRatio));
    }

    @Override
    public boolean turn(double degrees) {
        return set(getAngle() + degrees);
    }

    @Override
    public boolean set(double degrees) {
        if (!isAllowed(degrees)) {
            return false;
        }
        double target = MotorMath.resolveTarget(this.mode, getAngle(), degrees);
        this.controller.setSetpoint(
            MotorMath.mechanismDegreesToMotorRotations(target, this.gearRatio),
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );
        return true;
    }

    @Override
    public double getAngle() {
        return MotorMath.motorRotationsToMechanismDegrees(this.encoder.getPosition(), this.gearRatio);
    }

    @Override
    public double getSpeedRPM() {
        return MotorMath.motorRpmToMechanismRpm(this.encoder.getVelocity(), this.gearRatio);
    }

    @Override
    public void resetAngle(double degrees) {
        this.encoder.setPosition(MotorMath.mechanismDegreesToMotorRotations(degrees, this.gearRatio));
    }

    @Override
    public boolean isAtAngle(double targetDegrees, double toleranceDegrees) {
        return MotorMath.isAtAngle(this.mode, getAngle(), targetDegrees, toleranceDegrees);
    }

    /**
     * Vortex wrappers default to coasting (plain {@code stopMotor()}), because in 2026 they were
     * only used for rollers. Call {@link #disableCoasting()} to hold position on stop instead.
     */
    @Override
    public void stopTurning() {
        if (this.coastingEnabled) {
            this.motor.stopMotor();
        } else {
            holdCurrentPosition();
        }
    }

    @Override
    public void enableCoasting() {
        this.coastingEnabled = true;
        this.motor.stopMotor();
    }

    @Override
    public void disableCoasting() {
        this.coastingEnabled = false;
        holdCurrentPosition();
    }

    /** Escape hatch for anything the wrapper does not cover. */
    public SparkFlex getMotor() {
        return this.motor;
    }

    private void holdCurrentPosition() {
        this.controller.setSetpoint(this.encoder.getPosition(), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    private boolean isAllowed(double degrees) {
        if (!this.hasAngleLimits) {
            return true;
        }
        double checked = MotorMath.limitCheckAngle(this.mode, degrees);
        if (MotorMath.withinLimits(checked, this.minAngle, this.maxAngle)) {
            return true;
        }
        DriverStation.reportWarning(String.format(
            "%s: target %.2f deg is outside limits [%.2f, %.2f]", this.name, checked, this.minAngle, this.maxAngle
        ), false);
        return false;
    }

    public static class Builder {
        private int canId;
        private double gearRatio = 1.0;
        private double p;
        private double i;
        private double d;
        private boolean velocityGainsSet = false;
        private double velocityP;
        private double velocityI;
        private double velocityD;
        private double kS;
        private double kV;
        private double minOutput = -1.0;
        private double maxOutput = 1.0;
        private boolean hasAngleLimits = false;
        private double minAngle;
        private double maxAngle;
        private MotorMode mode = MotorMode.CONTINUOUS;
        private boolean inverted = false;
        private boolean brakeMode = true;
        private int smartCurrentLimit = 80;
        private double startingAngle = 0.0;

        public static Builder newInstance() { return new Builder(); }

        public Builder() {}

        public Builder canId(int canId) { this.canId = canId; return this; }
        /** Alias of {@link #canId} so Kraken and Vortex builders read the same. */
        public Builder port(int port) { this.canId = port; return this; }
        /** Motor rotations per mechanism rotation (e.g. 10 for a 10:1 reduction). */
        public Builder gearRatio(double gearRatio) { this.gearRatio = gearRatio; return this; }
        /** Position gains (slot 0). Also used for velocity unless {@link #velocityPID} is set. */
        public Builder PID(double p, double i, double d) { this.p = p; this.i = i; this.d = d; return this; }
        /** Velocity gains (slot 1). Error is in motor RPM. */
        public Builder velocityPID(double p, double i, double d) {
            this.velocityGainsSet = true;
            this.velocityP = p;
            this.velocityI = i;
            this.velocityD = d;
            return this;
        }
        /** Velocity feedforward (slot 1): kS in volts, kV in volts per motor RPM. */
        public Builder velocityFF(double kS, double kV) { this.kS = kS; this.kV = kV; return this; }
        /** Closed-loop output limits, clamped to [-1, 1]. Defaults to full range. */
        public Builder outputRange(double minOutput, double maxOutput) {
            this.minOutput = MotorMath.clampDutyCycle(minOutput);
            this.maxOutput = MotorMath.clampDutyCycle(maxOutput);
            return this;
        }
        /** Soft limits in mechanism degrees. Without this call the motor has no limits. */
        public Builder angleLimits(double minAngle, double maxAngle) {
            this.hasAngleLimits = true;
            this.minAngle = Math.min(minAngle, maxAngle);
            this.maxAngle = Math.max(minAngle, maxAngle);
            return this;
        }
        public Builder noAngleLimits() { this.hasAngleLimits = false; return this; }
        public Builder mode(MotorMode mode) { this.mode = mode; return this; }
        public Builder inverted(boolean inverted) { this.inverted = inverted; return this; }
        /** Brake (true, default) or coast (false) idle mode. */
        public Builder brakeMode(boolean brakeMode) { this.brakeMode = brakeMode; return this; }
        /** Smart current limit in amps. Defaults to 80, REV's recommendation for the Vortex. */
        public Builder smartCurrentLimit(int amps) { this.smartCurrentLimit = amps; return this; }
        /** Angle (mechanism degrees) the encoder is seeded with at boot. Defaults to 0. */
        public Builder startingAngle(double degrees) { this.startingAngle = degrees; return this; }

        public SmartVortexMotor build() { return new SmartVortexMotor(this); }
    }
}
