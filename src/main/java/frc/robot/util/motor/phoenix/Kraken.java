package frc.robot.util.motor.phoenix;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Pair;

public class Kraken extends TalonFX {

    private double targetPosition = 0.0;
    private double targetVelocity = 0.0;
    private double targetPercent = 0.0;

    TalonFXConfigurator configurator;

    private double positionConversionFactor = 1.0;
    private double velocityConversionFactor = 1.0;

    private final MotionMagicVoltage positionRequest;
    private final MotionMagicVelocityVoltage velocityRequest;

    List<Pair<Kraken, Boolean>> followers = new ArrayList<Pair<Kraken, Boolean>>();

    private int id;

    public Kraken(int id) {
        this(id, false, false);
    }

    public Kraken(int id, boolean inverted) {
        this(id, inverted, false);
    }

    public Kraken(int id, boolean inverted, boolean useFOC) {
        super(id);
        setInverted(inverted);
        configurator = getConfigurator();
        positionRequest = new MotionMagicVoltage(0, useFOC, 0, 0, false, false, false);
        velocityRequest = new MotionMagicVelocityVoltage(0, 0, useFOC, 0, 0, false, false, false);
        this.id = id;
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
        runFollowers();
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
        if (velocity == 0) {
            setVoltage(0);
        }
        runFollowers();
        targetVelocity = velocity;
    }

    @Override
    public void setVoltage(double voltage) {
        setVoltage(voltage);
        runFollowers();
    }

    public void setPercent(double percent) {
        set(percent);
        if (percent == 0) {
            setVoltage(0);
        }
        runFollowers();
        targetPercent = percent;
    }

    public void setPositionConversionFactor(double newFactor) {
        positionConversionFactor = newFactor;
    }

    public void setVelocityConversionFactor(double newFactor) {
        velocityConversionFactor = newFactor;
    }

    public void setBrakeMode() {
        MotorOutputConfigs config = new MotorOutputConfigs();
        config.NeutralMode = NeutralModeValue.Brake;
        configurator.apply(config);
    }

    public void setCoastMode() {
        MotorOutputConfigs config = new MotorOutputConfigs();
        config.NeutralMode = NeutralModeValue.Coast;
        configurator.apply(config);
    }

    public void addFollower(Kraken motor, boolean invert) {
        followers.add(Pair.of(motor, invert));
    }

    public void addFollower(Kraken motor) {
        addFollower(motor, false);
    }

    public void runFollowers() {
        for (Pair<Kraken, Boolean> pair : followers) {
            Kraken motor = pair.getFirst();
            boolean invert = pair.getSecond();
            motor.setControl(new Follower(id, invert));
        }
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

    public double getVoltageAsDouble() {
        return super.getMotorVoltage().refresh().getValue();
    }

    public int getID() {
        return id;
    }
}
