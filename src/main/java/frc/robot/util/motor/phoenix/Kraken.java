package frc.robot.util.motor.phoenix;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Kraken extends TalonFX {

    private double targetPosition = 0.0;
    private double targetVelocity = 0.0;
    private double targetPercent = 0.0;

    private double positionConversionFactor = 1.0;
    private double velocityConversionFactor = 1.0;

    private final TalonFXConfiguration talonFXConfigs;

    private final MotionMagicVoltage positionRequest;
    private final MotionMagicVelocityVoltage velocityRequest;

    public Kraken(int id) {
        this(id, false, false);
    }

    public Kraken(int id, boolean inverted) {
        this(id, inverted, false);
    }

    public Kraken(int id, boolean inverted, boolean useFOC) {
        super(id);
        setInverted(inverted);
        talonFXConfigs = new TalonFXConfiguration();
        positionRequest = new MotionMagicVoltage(0, useFOC, 0, 0, false, false, false);
        velocityRequest = new MotionMagicVelocityVoltage(0, 0, useFOC, 0, 0, false, false, false);
    }

    public void setTargetPosition(double position) {
        setTargetPosition(position, 0, 0);
    }

    public void setTargetPosition(double position, double feedForward) {
        setTargetPosition(position, feedForward, 0);
    }

    public void setTargetPosition(double position, int slot) {
        setTargetPosition(position, 0, slot);
    }

    public void setTargetPosition(double position, double feedForward, int slot) {
        setControl(
            positionRequest
                .withPosition(position / positionConversionFactor)
                .withFeedForward(feedForward)
                .withSlot(slot));
        targetPosition = position;
    }

    public void setTargetVelocity(double velocity) {
        setTargetVelocity(velocity, 0, 0);
    }

    public void setTargetVelocity(double velocity, double feedForward) {
        setTargetVelocity(velocity, feedForward, 0);
    }

    public void setTargetVelocity(double velocity, int slot) {
        setTargetVelocity(velocity, 0, slot);
    }

    public void setTargetVelocity(double velocity, double feedForward, int slot) {
        setControl(
            velocityRequest
                .withVelocity(velocity / velocityConversionFactor)
                .withFeedForward(feedForward)
                .withSlot(slot));
        targetVelocity = velocity;
    }

    public void setPercent(double percent) {
        set(percent);
        targetPercent = percent;
    }

    public void setPositionConversionFactor(double newFactor) {
        positionConversionFactor = newFactor;
    }

    public void setVelocityConversionFactor(double newFactor) {
        velocityConversionFactor = newFactor;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getTargetPercent() {
        return targetPercent;
    }

    public double getPositionAsDouble() {
        return super.getPosition().refresh().getValue() * positionConversionFactor;        
    }

    public double getVelocityAsDouble() {
        return super.getVelocity().refresh().getValue() * velocityConversionFactor;
    }
}
