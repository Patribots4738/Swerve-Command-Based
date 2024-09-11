package frc.robot.util.motor.phoenix;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.KrakenMotorConstants;
import frc.robot.util.custom.PatrIDConstants;

public class Kraken extends TalonFX {

    private double targetPosition = 0.0;
    private double targetVelocity = 0.0;
    private double targetPercent = 0.0;

    private TalonFXConfigurator configurator;

    private double positionConversionFactor = 1.0;
    private double velocityConversionFactor = 1.0;

    private final PositionVoltage positionRequest;
    private final VelocityVoltage velocityRequest;

    List<Pair<Kraken, Boolean>> followers = new ArrayList<Pair<Kraken, Boolean>>();

    private int canID;
    private boolean useFOC;

    private ControlLoopType controlType = ControlLoopType.PERCENT;

    private DCMotorSim motorSimModel;

    public Kraken(int id) {
        this(id, false, false);
    }

    public Kraken(int id, boolean inverted) {
        this(id, inverted, false);
    }

    public Kraken(int id, boolean inverted, boolean useFOC) {
        super(id);
        this.canID = id;
        this.useFOC = useFOC;
        configurator = getConfigurator();
        positionRequest = new PositionVoltage(0).withEnableFOC(useFOC);
        velocityRequest = new VelocityVoltage(0).withEnableFOC(useFOC);
        setInverted(inverted);
        setBrakeMode();
        register();
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
        controlType = ControlLoopType.POSITION;
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
        controlType = ControlLoopType.VELOCITY;
    }

    @Override
    public void setVoltage(double voltage) {
        setVoltage(voltage);
        runFollowers();
    }

    @Override
    public void set(double percent) {
        set(percent);
        if (percent == 0) {
            setVoltage(0);
        }
        runFollowers();
        targetPercent = percent;
        controlType = ControlLoopType.PERCENT;
    }

    public void setPositionConversionFactor(double newFactor) {
        positionConversionFactor = newFactor;
    }

    public void setVelocityConversionFactor(double newFactor) {
        velocityConversionFactor = newFactor;
    }

    public void setSupplyCurrentLimit(double currentLimit) {
        CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
        configs.SupplyCurrentLimit = currentLimit;
        configs.SupplyCurrentLimitEnable = true;
        configurator.apply(configs);
    }

    public void setStatorCurrentLimit(double currentLimit) {
        CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
        configs.StatorCurrentLimit = currentLimit;
        configs.StatorCurrentLimitEnable = true;
        configurator.apply(configs);
    }

    public void resetEncoder(double position) {
        setPosition(position);
    }

    public void resetEncoder() {
        setPosition(0);
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
            motor.setControl(new Follower(canID, invert));
        }
    }

    public double getPositionConversionFactor() {
        return positionConversionFactor;
    }

    public double getVelocityConversionFactor() {
        return velocityConversionFactor;
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

    public double getSupplyCurrentAsDouble() {
        return super.getSupplyCurrent().refresh().getValue();
    }

    public double getStatorCurrentAsDouble() {
        return super.getStatorCurrent().refresh().getValue();
    }

    public double getTorqueCurrentAsDouble() {
        return super.getTorqueCurrent().refresh().getValue();
    }

    public int getCANID() {
        return canID;
    }

    public void register() {
        KrakenMotorConstants.KRAKEN_MOTOR_MAP.put(canID, this);

        if (FieldConstants.IS_SIMULATION) {
            motorSimModel = new DCMotorSim(useFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1), 1, 0.001);
        }
    }

    public void tick() {
        TalonFXSimState sim = getSimState();

        sim.setSupplyVoltage(RobotController.getBatteryVoltage());

        double motorVoltage = sim.getMotorVoltage();
  
        motorSimModel.setInputVoltage(motorVoltage);
        motorSimModel.update(0.020);

        sim.setRawRotorPosition(motorSimModel.getAngularPositionRotations());
        sim.setRotorVelocity(Units.radiansToRotations(motorSimModel.getAngularVelocityRadPerSec()));
    }

    public void applySlot0Configs(Slot0Configs configs) {
        configurator.apply(configs);
    }

    public void applySlot1Configs(Slot1Configs configs) {
        configurator.apply(configs);
    }

    public void applySlot2Configs(Slot2Configs configs) {
        configurator.apply(configs);
    }

    public void applySlotConfigs(SlotConfigs configs, int slot) {
        configs.SlotNumber = slot;
        configurator.apply(configs);
    }

    // Velocity control gains
    public void applyGainConfigs(double P, double I, double D, double S, double V, double G, int slot)  {
        SlotConfigs configs = new SlotConfigs();
        configs.kP = P;
        configs.kI = I;
        configs.kD = D;
        configs.kS = S;
        configs.kV = V;
        applySlotConfigs(configs, slot);
    }

    public void applyGainConfigs(double P, double I, double D, double S, double V, double G) {
        applyGainConfigs(P, I, D, S, V, G, 0);
    }

    // Position control gains
    public void setPID(double P, double I, double D, int slot) {
        SlotConfigs configs = new SlotConfigs();
        configs.kP = P;
        configs.kI = I;
        configs.kD = D;
        applySlotConfigs(configs, slot);
    }

    public void setPID(double P, double I, double D) {
        setPID(P, I , D, 0);
    }

    public void setPID(PatrIDConstants constants, int slot) {
        setPID(constants.getP(), constants.getI(), constants.getD(), slot);
    }

    public void setPID(PatrIDConstants constants) {
        setPID(constants, 0);
    }

    public enum ControlLoopType {
        POSITION,
        VELOCITY,
        PERCENT;
    }

}
