package frc.robot.util.motor.phoenix;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
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

    private final TalonFXConfigurator configurator;
    private final TalonFXSimState sim;

    private DCMotorSim motorSimModel;

    private double positionConversionFactor = 1.0;
    private double velocityConversionFactor = 1.0;

    private final PositionVoltage positionRequest;
    private final VelocityVoltage velocityRequest;

    private boolean useFOC;

    private ControlLoopType controlType = ControlLoopType.PERCENT;

    /**
     * Creates new Kraken motor.
     * 
     * @param id ID of Kraken motor
     */
    public Kraken(int id) {
        this(id, false, false);
    }

    /**
     * Creates new Kraken motor that can be inverted.
     * 
     * @param id ID of Kraken motor
     * @param inverted inverts input given to motor when set to true
     */
    public Kraken(int id, boolean inverted) {
        this(id, inverted, false);
    }

    /**
     * Creates a new Kraken motor that can be inverted and use FOC.
     * 
     * @param id ID of Kraken motor
     * @param inverted inverts input given to motor when set to true
     * @param useFOC uses FOC to enhance motor communication when set to true
     */
    public Kraken(int id, boolean inverted, boolean useFOC) {
        super(id);
        this.useFOC = useFOC;
        configurator = getConfigurator();
        sim = getSimState();
        positionRequest = new PositionVoltage(0).withEnableFOC(useFOC);
        velocityRequest = new VelocityVoltage(0).withEnableFOC(useFOC);
        setInverted(inverted);
        setBrakeMode();
        register();
    }

    /**
     * Sets the target position of the Kraken.
     * 
     * @param position motor position
     */
    public void setTargetPosition(double position) {
        setTargetPosition(position, 0, 0);
    }

    /**
     * Sets the target position of the Kraken and feed forward control.
     * 
     * @param position motor position
     * @param feedForward feed forward control
     */
    public void setTargetPosition(double position, double feedForward) {
        setTargetPosition(position, feedForward, 0);
    }

    /**
     * Sets the target position of the Kraken and PID slot
     * 
     * @param position motor position
     * @param slot PID slot
     */
    public void setTargetPosition(double position, int slot) {
        setTargetPosition(position, 0, slot);
    }

    /**
     * Sets the target position of the Kraken, feed forward control and PID slot
     * 
     * @param position motor position
     * @param feedForward feed forward control
     * @param slot PID slot
     */
    public void setTargetPosition(double position, double feedForward, int slot) {
        setControl(
            positionRequest
                .withPosition(position / positionConversionFactor)
                .withFeedForward(feedForward)
                .withSlot(slot));
        targetPosition = position;
        controlType = ControlLoopType.POSITION;
    }

    /**
     * Sets target velocity of the Kraken (mps?)
     * 
     * @param velocity target velocity
     */
    public void setTargetVelocity(double velocity) {
        setTargetVelocity(velocity, 0, 0);
    }

    /**
     * Sets the target velocity of the Kraken and feed forward control
     * 
     * @param velocity target velocity
     * @param feedForward feed forward control
     */
    public void setTargetVelocity(double velocity, double feedForward) {
        setTargetVelocity(velocity, feedForward, 0);
    }

    /**
     * Sets the target velocity of the Kraken and PID slot
     * 
     * @param velocity target velocity
     * @param slot PID slot
     */
    public void setTargetVelocity(double velocity, int slot) {
        setTargetVelocity(velocity, 0, slot);
    }

    /**
     * Sets the target velocity of the Kraken, feed forward control, and PID slot
     * 
     * @param velocity target velocity
     * @param feedForward feed forward control
     * @param slot PID slot
     */
    public void setTargetVelocity(double velocity, double feedForward, int slot) {
        setControl(
            velocityRequest
                .withVelocity(velocity / velocityConversionFactor)
                .withFeedForward(feedForward)
                .withSlot(slot));
        if (velocity == 0) {
            setVoltage(0);
        }
        targetVelocity = velocity;
        controlType = ControlLoopType.VELOCITY;
    }

    /**
     * Sets the speed of Kraken using percent of its maximum output.
     * 
     * @param percent should be between 1.0 to -1.0
     */
    @Override
    public void set(double percent) {
        super.set(percent);
        if (percent == 0) {
            setVoltage(0);
        }
        targetPercent = percent;
        controlType = ControlLoopType.PERCENT;
    }

    /**
     * Sets the speed of the Kraken to 0.
     */
    @Override 
    public void stopMotor() {
        set(0);
    }

    /**
     * Sets new conversion factor for the position of the Kraken.
     * 
     * @param newFactor new conversion factor
     */
    public void setPositionConversionFactor(double newFactor) {
        positionConversionFactor = newFactor;
    }

    /**
     * Sets new conversion factor for the velocity of the Kraken.
     * 
     * @param newFactor new conversion factor
     */
    public void setVelocityConversionFactor(double newFactor) {
        velocityConversionFactor = newFactor;
    }

    /**
     * Sets the limit of current that can be supplied to the Kraken.
     * 
     * @param currentLimit maximum allowable current
     */
    public void setSupplyCurrentLimit(double currentLimit) {
        CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
        configs.SupplyCurrentLimit = currentLimit;
        configs.SupplyCurrentLimitEnable = true;
        configurator.apply(configs);
    }

    /**
     * Sets the limit of the current supplied to the stator of the Kraken.
     * 
     * @param currentLimit maximum allowable current
     */
    public void setStatorCurrentLimit(double currentLimit) {
        CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
        configs.StatorCurrentLimit = currentLimit;
        configs.StatorCurrentLimitEnable = true;
        configurator.apply(configs);
    }

    /**
     * Resets encoder postion.
     * 
     * @param position position encoder should be set to
     */
    public void resetEncoder(double position) {
        setPosition(position);
    }

    /**
     * Resets the encoder position to 0.
     */
    public void resetEncoder() {
        setPosition(0);
    }

    /**
     * Puts the Kraken in brake mode and it can not move.
     */
    public void setBrakeMode() {
        MotorOutputConfigs config = new MotorOutputConfigs();
        config.NeutralMode = NeutralModeValue.Brake;
        configurator.apply(config);
    }

    /**
     * Puts the Kraken in coast mode and the motor can spin freely.
     */
    public void setCoastMode() {
        MotorOutputConfigs config = new MotorOutputConfigs();
        config.NeutralMode = NeutralModeValue.Coast;
        configurator.apply(config);
    }

    /**
     * Sets the ID of the Kraken's encoder.
     * 
     * @param canCoderId new encoder IF
     */
    public void setEncoder(int canCoderId) {
        FeedbackConfigs configs = new FeedbackConfigs();
        configs.FeedbackRemoteSensorID = canCoderId;
        configs.SensorToMechanismRatio = 1.0;
        configurator.apply(configs);
    }

    /**
     * Enables PID wrapping.
     * 
     * @param enabled set to true if wrapping is enabled
     */
    public void setPositionPIDWrappingEnabled(boolean enabled) {
        ClosedLoopGeneralConfigs configs = new ClosedLoopGeneralConfigs();
        configs.ContinuousWrap = enabled;
        configurator.apply(configs);
    }

    /**
     * Adds a follower motor to the Kraken.
     * 
     * @param motor follower motor
     * @param invert whether the follower motor is to be inverted
     */
    public void addFollower(Kraken motor, boolean invert) {
        motor.setControl(new Follower(getDeviceID(), invert));
    }

    /**
     * Adds a follower motor to the Kraken.
     * 
     * @param motor
     */
    public void addFollower(Kraken motor) {
        addFollower(motor, false);
    }

    /**
     * Obtains position conversion factor of the Kraken.
     * 
     * @return position conversion factor
     */
    public double getPositionConversionFactor() {
        return positionConversionFactor;
    }

    /**
     * Obtains velocity conversion factor of the Kraken.
     * 
     * @return velocity conversion factor
     */
    public double getVelocityConversionFactor() {
        return velocityConversionFactor;
    }

    /**
     * Obtains target position of the Kraken.
     * 
     * @return target position
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * Obtains target velocity of the Kraken.
     * 
     * @return target velocity
     */
    public double getTargetVelocity() {
        return targetVelocity;
    }

    /**
     * Obtains target percent speed of the Kraken.
     * 
     * @return target percent speed
     */
    public double getTargetPercent() {
        return targetPercent;
    }

    /**
     * Obtains the current position of the Kraken as a double.
     * 
     * @return current position
     */
    public double getPositionAsDouble() {
        return super.getPosition().refresh().getValue() * positionConversionFactor;        
    }

    /**
     * Obtains the current velocity of the Kraken as a double.
     * 
     * @return current velocity
     */
    public double getVelocityAsDouble() {
        return super.getVelocity().refresh().getValue() * velocityConversionFactor;
    }

    /**
     * Obtains the current voltage of the Kraken as a double.
     * 
     * @return current voltage
     */
    public double getVoltageAsDouble() {
        return super.getMotorVoltage().refresh().getValue();
    }

    /**
     * Represents current supplied to Kraken as a double.
     * 
     * @return supply of current to Kraken
     */
    public double getSupplyCurrentAsDouble() {
        return super.getSupplyCurrent().refresh().getValue();
    }

    /**
     * Represents current supplied to the stator of the Kraken as a double.
     * 
     * @return supply of current to stator
     */
    public double getStatorCurrentAsDouble() {
        return super.getStatorCurrent().refresh().getValue();
    }

    /**
     * Represents the current creating torque as a double.
     * 
     * @return torque current
     */
    public double getTorqueCurrentAsDouble() {
        return super.getTorqueCurrent().refresh().getValue();
    }

    /**
     * Adds the Kraken to motor map.
     */
    public void register() {
        KrakenMotorConstants.KRAKEN_MOTOR_MAP.put(getDeviceID(), this);

        if (FieldConstants.IS_SIMULATION) {
            motorSimModel = new DCMotorSim(useFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1), 1, 0.001);
        }
    }

    /**
     * Updates voltage, position and velocity in sim.
     */
    public void tick() {

        sim.setSupplyVoltage(RobotController.getBatteryVoltage());

        double motorVoltage = sim.getMotorVoltage();
  
        motorSimModel.setInputVoltage(motorVoltage);
        motorSimModel.update(0.020);

        sim.setRawRotorPosition(motorSimModel.getAngularPositionRotations());
        sim.setRotorVelocity(Units.radiansToRotations(motorSimModel.getAngularVelocityRadPerSec()));

    }

    public void applySlot0Gains(Slot0Configs configs) {
        configurator.apply(configs);
    }

    public void applySlot1Gains(Slot1Configs configs) {
        configurator.apply(configs);
    }

    public void applySlot2Gains(Slot2Configs configs) {
        configurator.apply(configs);
    }

    public void applySlotGains(SlotConfigs configs, int slot) {
        configs.SlotNumber = slot;
        configurator.apply(configs);
    }

    /**
     * Sets the gains for the specified slot.
     * 
     * @param P proportional gains
     * @param I integral gains
     * @param D derivative gains
     * @param S static feed forward gain
     * @param V velocity feed forward gain
     * @param G gravity feed forward / feedback gain
     * @param slot slot for gains to be added to 
     */
    public void setGains(double P, double I, double D, double S, double V, double G, int slot)  {
        SlotConfigs configs = new SlotConfigs();
        configs.kP = P;
        configs.kI = I;
        configs.kD = D;
        configs.kS = S;
        configs.kV = V;
        applySlotGains(configs, slot);
    }

    /**
     * Sets the gains for slot 0.
     * 
     * @param P proportional gains
     * @param I integral gains
     * @param D derivative gains
     * @param S static feed forward gain
     * @param V velocity feed forward gain
     * @param G gravity feed forward / feedback gain
     */
    public void setGains(double P, double I, double D, double S, double V, double G) {
        setGains(P, I, D, S, V, G, 0);
    }

    /**
     * Sets the gains for slot 0.
     * 
     * @param P proportional gains
     * @param I integral gains
     * @param D derivative gains
     * @param S static feed forward gain
     * @param V velocity feed forward gain
     */
    public void setGains(double P, double I, double D, double S, double V) {
        setGains(P, I, D, S, V, 0, 0);
    }

    /**
     * Sets the PID gains for the specified slot.
     * 
     * @param P proportional gains
     * @param I integral gains
     * @param D derivative gains
     * @param slot slot for gains to be added to 
     */
    public void setPID(double P, double I, double D, int slot) {
        SlotConfigs configs = new SlotConfigs();
        configs.kP = P;
        configs.kI = I;
        configs.kD = D;
        applySlotGains(configs, slot);
    }

    /**
     * Sets the PID gains for the slot 0.
     * 
     * @param P proportional gains
     * @param I integral gains
     * @param D derivative gains
     */
    public void setPID(double P, double I, double D) {
        setPID(P, I , D, 0);
    }

    /**
     * Sets the PID gains for the specified slot using PatriIDConstants.
     * 
     * @param constants PID constants from PatriIDConstants
     * @param slot slot for gains to be added to 
     */
    public void setPID(PatrIDConstants constants, int slot) {
        setPID(constants.getP(), constants.getI(), constants.getD(), slot);
    }

    /**
     * Sets the PID gains for the slot 0 using PatriIDConstants.
     * 
     * @param constants PID constants from PatriIDConstants
     */
    public void setPID(PatrIDConstants constants) {
        setPID(constants, 0);
    }

    public enum ControlLoopType {
        POSITION,
        VELOCITY,
        PERCENT;
    }

}
