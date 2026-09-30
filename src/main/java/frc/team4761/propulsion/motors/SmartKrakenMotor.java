package frc.team4761.propulsion.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * A Kraken X60 / X44 (or any TalonFX) with mechanism-unit helpers.
 *
 * <pre>{@code
 * SmartKrakenMotor pivot = SmartKrakenMotor.Builder.newInstance()
 *     .port(55)
 *     .PID(0.5, 0.0, 0.0)          // position gains (slot 0)
 *     .gearRatio(13.1875)          // motor rotations per mechanism rotation
 *     .angleLimits(-105.0, 0.0)    // optional soft limits, mechanism degrees
 *     .brakeMode(true)
 *     .statorCurrentLimit(60.0)
 *     .build();
 * }</pre>
 *
 * <p>Position control uses slot 0. Velocity control uses slot 1; if you never call
 * {@link Builder#velocityPID} the position gains are copied into slot 1 so older code that only
 * called {@code PID(...)} keeps working.
 *
 * <p>Gains are in duty-cycle units (PositionDutyCycle / VelocityDutyCycle), matching our
 * 2026 tuning.
 */
public class SmartKrakenMotor implements SmartMotor {
    private static final int POSITION_SLOT = 0;
    private static final int VELOCITY_SLOT = 1;

    private final TalonFX motor;
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0).withSlot(POSITION_SLOT);
    private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0).withSlot(VELOCITY_SLOT);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    private final double gearRatio;
    private final boolean hasAngleLimits;
    private final double minAngle;
    private final double maxAngle;
    private final MotorMode mode;
    private final String name;
    private boolean coastingEnabled = false;

    public SmartKrakenMotor(Builder builder) {
        MotorMath.requirePositive(builder.gearRatio, "gearRatio");
        this.motor = new TalonFX(builder.port, new CANBus(builder.canBus));
        this.gearRatio = builder.gearRatio;
        this.hasAngleLimits = builder.hasAngleLimits;
        this.minAngle = builder.minAngle;
        this.maxAngle = builder.maxAngle;
        this.mode = builder.mode;
        this.name = "SmartKrakenMotor[" + builder.port + "]";

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = builder.p;
        config.Slot0.kI = builder.i;
        config.Slot0.kD = builder.d;

        config.Slot1.kP = builder.velocityGainsSet ? builder.velocityP : builder.p;
        config.Slot1.kI = builder.velocityGainsSet ? builder.velocityI : builder.i;
        config.Slot1.kD = builder.velocityGainsSet ? builder.velocityD : builder.d;
        config.Slot1.kS = builder.kS;
        config.Slot1.kV = builder.kV;

        config.MotorOutput.PeakForwardDutyCycle = builder.maxOutput;
        config.MotorOutput.PeakReverseDutyCycle = builder.minOutput;
        config.MotorOutput.Inverted = builder.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = builder.brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        if (builder.statorCurrentLimit > 0.0) {
            config.CurrentLimits.StatorCurrentLimit = builder.statorCurrentLimit;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
        }

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int attempt = 0; attempt < 3 && !status.isOK(); attempt++) {
            status = this.motor.getConfigurator().apply(config);
        }
        if (!status.isOK()) {
            DriverStation.reportError(name + " failed to apply config: " + status, false);
        }

        this.motor.setPosition(MotorMath.mechanismDegreesToMotorRotations(builder.startingAngle, gearRatio));
    }

    @Override
    public void setSpeedPercent(double percent) {
        this.motor.setControl(this.dutyCycleRequest.withOutput(MotorMath.clampDutyCycle(percent)));
    }

    /** Kept for compatibility with 2026 code; same as {@link #setSpeedPercent}. */
    public void setRawSpeedPercent(double percent) {
        setSpeedPercent(percent);
    }

    @Override
    public void setRawSpeed(double motorRpm) {
        this.motor.setControl(this.velocityRequest.withVelocity(motorRpm / 60.0));
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
        this.motor.setControl(
            this.positionRequest.withPosition(MotorMath.mechanismDegreesToMotorRotations(target, this.gearRatio))
        );
        return true;
    }

    @Override
    public double getAngle() {
        return MotorMath.motorRotationsToMechanismDegrees(this.motor.getPosition().getValueAsDouble(), this.gearRatio);
    }

    @Override
    public double getSpeedRPM() {
        return MotorMath.motorRpmToMechanismRpm(this.motor.getVelocity().getValueAsDouble() * 60.0, this.gearRatio);
    }

    @Override
    public void resetAngle(double degrees) {
        this.motor.setPosition(MotorMath.mechanismDegreesToMotorRotations(degrees, this.gearRatio));
    }

    @Override
    public boolean isAtAngle(double targetDegrees, double toleranceDegrees) {
        return MotorMath.isAtAngle(this.mode, getAngle(), targetDegrees, toleranceDegrees);
    }

    @Override
    public void stopTurning() {
        if (this.coastingEnabled) {
            this.motor.setControl(this.dutyCycleRequest.withOutput(0.0));
        } else {
            holdCurrentPosition();
        }
    }

    @Override
    public void enableCoasting() {
        this.coastingEnabled = true;
        this.motor.setControl(this.dutyCycleRequest.withOutput(0.0));
    }

    @Override
    public void disableCoasting() {
        this.coastingEnabled = false;
        holdCurrentPosition();
    }

    /** Escape hatch for anything the wrapper does not cover (signals, followers, sim state...). */
    public TalonFX getMotor() {
        return this.motor;
    }

    private void holdCurrentPosition() {
        this.motor.setControl(this.positionRequest.withPosition(this.motor.getPosition().getValueAsDouble()));
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
        private int port;
        private String canBus = "rio";
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
        private double gearRatio = 1.0;
        private boolean inverted = false;
        private boolean brakeMode = true;
        private double statorCurrentLimit = 0.0;
        private double startingAngle = 0.0;

        public static Builder newInstance() { return new Builder(); }

        public Builder() {}

        /** CAN ID of the TalonFX. */
        public Builder port(int port) { this.port = port; return this; }
        /** CAN bus name: "rio" (default) or the name of a CANivore. */
        public Builder canBus(String canBus) { this.canBus = canBus; return this; }
        /** Position gains (slot 0). Also used for velocity unless {@link #velocityPID} is set. */
        public Builder PID(double p, double i, double d) { this.p = p; this.i = i; this.d = d; return this; }
        /** Velocity gains (slot 1). */
        public Builder velocityPID(double p, double i, double d) {
            this.velocityGainsSet = true;
            this.velocityP = p;
            this.velocityI = i;
            this.velocityD = d;
            return this;
        }
        /** Velocity feedforward (slot 1): kS = static friction, kV = output per rotor rotation/sec. */
        public Builder velocityFF(double kS, double kV) { this.kS = kS; this.kV = kV; return this; }
        /** Peak duty cycle limits, clamped to [-1, 1]. Defaults to full range. */
        public Builder outputRange(double minOutput, double maxOutput) {
            this.minOutput = MotorMath.clampDutyCycle(minOutput);
            this.maxOutput = MotorMath.clampDutyCycle(maxOutput);
            return this;
        }
        /**
         * Soft limits in mechanism degrees. Without this call the motor has no limits.
         * In WRAPPED mode the limits are compared against the angle wrapped into [0, 360).
         */
        public Builder angleLimits(double minAngle, double maxAngle) {
            this.hasAngleLimits = true;
            this.minAngle = Math.min(minAngle, maxAngle);
            this.maxAngle = Math.max(minAngle, maxAngle);
            return this;
        }
        /** Removes any soft limits. This is the default. */
        public Builder noAngleLimits() { this.hasAngleLimits = false; return this; }
        public Builder mode(MotorMode mode) { this.mode = mode; return this; }
        /** Motor rotations per mechanism rotation (e.g. 10 for a 10:1 reduction). */
        public Builder gearRatio(double gearRatio) { this.gearRatio = gearRatio; return this; }
        public Builder inverted(boolean inverted) { this.inverted = inverted; return this; }
        /** Brake (true, default) or coast (false) when the output is neutral. */
        public Builder brakeMode(boolean brakeMode) { this.brakeMode = brakeMode; return this; }
        /** Stator current limit in amps. 0 (default) leaves the Phoenix default in place. */
        public Builder statorCurrentLimit(double amps) { this.statorCurrentLimit = amps; return this; }
        /** Angle (mechanism degrees) the encoder is seeded with at boot. Defaults to 0. */
        public Builder startingAngle(double degrees) { this.startingAngle = degrees; return this; }

        public SmartKrakenMotor build() { return new SmartKrakenMotor(this); }
    }
}
