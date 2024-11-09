package frc.robot.util.hardware.phoenix;

import com.ctre.phoenix6.StatusSignal;
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
import frc.robot.util.custom.GainConstants;

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

    private StatusSignal<Double> positionSignal;
    private StatusSignal<Double> velocitySignal;
    private StatusSignal<Double> voltageSignal;
    private StatusSignal<Double> supplyCurrentSignal;
    private StatusSignal<Double> statorCurrentSignal;   
    private StatusSignal<Double> torqueCurrentSignal;

    private final Slot0Configs slot0Configs = new Slot0Configs();
    private final Slot1Configs slot1Configs = new Slot1Configs();
    private final Slot2Configs slot2Configs = new Slot2Configs();

    private boolean useFOC;

    /**
     * Creates new Kraken motor.
     * 
     * @param id ID of Kraken motor
     * @param canBus CANivore Kraken is connected to
     */
    public Kraken(int id, String canBus) {
        this(id, canBus, false, false);
    }

    /**
     * Creates new Kraken motor.
     * 
     * @param id ID of Kraken motor
     */
    public Kraken(int id) {
        this(id, "rio");
    }

    /**
     * Creates new Kraken motor that can be inverted.
     * 
     * @param id ID of Kraken motor
     * @param canBus CANivore Kraken is connected to
     * @param inverted inverts input given to motor when set to true
     */
    public Kraken(int id, String canBus, boolean inverted) {
        this(id, canBus, inverted, false);
    }

    /**
     * Creates new Kraken motor that can be inverted.
     * 
     * @param id ID of Kraken motor
     * @param inverted inverts input given to motor when set to true
     */
    public Kraken(int id, boolean inverted) {
        this(id, "rio", inverted);
    }

    /**
     * Creates a new Kraken motor that can be inverted and use FOC.
     * 
     * @param id ID of Kraken motor
     * @param inverted inverts input given to motor when set to true
     * @param useFOC uses FOC to enhance motor communication when set to true
     */
    public Kraken(int id, boolean inverted, boolean useFOC) {
        this(id, "rio", inverted, useFOC);
    }

    /**
     * Creates a new Kraken motor that can be inverted and use FOC.
     * 
     * @param id ID of Kraken motor
     * @param inverted inverts input given to motor when set to true
     * @param canBus CANivore Kraken is connected to
     * @param useFOC uses FOC to enhance motor communication when set to true
     */
    public Kraken(int id, String canBus, boolean inverted, boolean useFOC) {
        super(id, canBus);
        this.useFOC = useFOC;
        configurator = getConfigurator();
        sim = getSimState();
        positionRequest = new PositionVoltage(0).withEnableFOC(useFOC);
        velocityRequest = new VelocityVoltage(0).withEnableFOC(useFOC);
        setStatusSignals();
        // optimizeBusUtilization();
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
    }

    /**
     * Sets target velocity of the Kraken (rps)
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
        return super.getPosition().getValue() * positionConversionFactor;        
    }

    /**
     * Obtains the current velocity of the Kraken as a double.
     * 
     * @return current velocity
     */
    public double getVelocityAsDouble() {
        return super.getVelocity().getValue() * velocityConversionFactor;
    }

    /**
     * Obtains the current voltage of the Kraken as a double.
     * 
     * @return current voltage
     */
    public double getVoltageAsDouble() {
        return super.getMotorVoltage().getValue();
    }

    /**
     * Represents current supplied to Kraken as a double.
     * 
     * @return supply of current to Kraken
     */
    public double getSupplyCurrentAsDouble() {
        return super.getSupplyCurrent().getValue();
    }

    /**
     * Represents current supplied to the stator of the Kraken as a double.
     * 
     * @return supply of current to stator
     */
    public double getStatorCurrentAsDouble() {
        return super.getStatorCurrent().getValue();
    }

    /**
     * Represents the current creating torque as a double.
     * 
     * @return torque current
     */
    public double getTorqueCurrentAsDouble() {
        return super.getTorqueCurrent().getValue();
    }

    public void setStatusSignals() {
        positionSignal = super.getPosition();
        velocitySignal = super.getVelocity();
        voltageSignal = super.getMotorVoltage();
        supplyCurrentSignal = super.getSupplyCurrent();
        statorCurrentSignal = super.getStatorCurrent();
        torqueCurrentSignal = super.getTorqueCurrent();
    }

    public void refreshStatusSignals() {
        positionSignal.refresh();
        velocitySignal.refresh();
        voltageSignal.refresh();
        supplyCurrentSignal.refresh();
        statorCurrentSignal.refresh();
        torqueCurrentSignal.refresh();
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
        if (slot == 0) {
            slot0Configs.kP = P;
            slot0Configs.kI = I;
            slot0Configs.kD = D;
            slot0Configs.kS = S;
            slot0Configs.kV = V;
            applySlot0Gains(slot0Configs);
        } else if (slot == 1) {
            slot1Configs.kP = P;
            slot1Configs.kI = I;
            slot1Configs.kD = D;
            slot1Configs.kS = S;
            slot1Configs.kV = V;
            applySlot1Gains(slot1Configs);
        } else {
            slot2Configs.kP = P;
            slot2Configs.kI = I;
            slot2Configs.kD = D;
            slot2Configs.kS = S;
            slot2Configs.kV = V;
            applySlot2Gains(slot2Configs);
        }
        
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

    public void setGains(GainConstants constants, int slot) {
        setGains(constants.getP(), constants.getI(), constants.getD(), constants.getS(), constants.getV(), constants.getG(), slot);
    }

    public void setGains(GainConstants constants) {
        setGains(constants.getP(), constants.getI(), constants.getD(), constants.getS(), constants.getV(), constants.getG());
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
        setGains(P, I, D, 0, 0, 0, slot);
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
     * Sets the PID gains for the specified slot using GainConstants.
     * 
     * @param constants PID constants from GainConstants
     * @param slot slot for gains to be added to 
     */
    public void setPID(GainConstants constants, int slot) {
        setPID(constants.getP(), constants.getI(), constants.getD(), slot);
    }

    /**
     * Sets the PID gains for the slot 0 using GainConstants.
     * 
     * @param constants PID constants from GainConstants
     */
    public void setPID(GainConstants constants) {
        setPID(constants, 0);
    }

    public void setP(double P) {
        slot0Configs.kP = P;
        applySlot0Gains(slot0Configs);
    }

    public void setI(double I) {
        slot0Configs.kI = I;
        applySlot0Gains(slot0Configs);
    }

    public void setD(double D) {
        slot0Configs.kD = D;
        applySlot0Gains(slot0Configs);
    }

    public void setS(double S) {
        slot0Configs.kS = S;
        applySlot0Gains(slot0Configs);
    }

    public void setV(double V) {
        slot0Configs.kV = V;
        applySlot0Gains(slot0Configs);
    }

    public void setG(double G) {
        slot0Configs.kG = G;
        applySlot0Gains(slot0Configs);
    }

    public double getP(int slot) {
        if (slot == 0) {
            return slot0Configs.kP;
        } else if (slot == 1) {
            return slot1Configs.kP;
        }
        return slot2Configs.kP;
    }

    public double getP() {
        return getP(0);
    }

    public double getI(int slot) {
        if (slot == 0) {
            return slot0Configs.kI;
        } else if (slot == 1) {
            return slot1Configs.kI;
        }
        return slot2Configs.kI;
    }

    public double getI() {
        return getI(0);
    }

    public double getD(int slot) {
        if (slot == 0) {
            return slot0Configs.kD;
        } else if (slot == 1) {
            return slot1Configs.kD;
        }
        return slot2Configs.kD;
    }

    public double getD() {
        return getD(0);
    }

    public double getS(int slot) {
        if (slot == 0) {
            return slot0Configs.kS;
        } else if (slot == 1) {
            return slot1Configs.kS;
        }
        return slot2Configs.kS;
    }

    public double getS() {
        return getS(0);
    }

    public double getV(int slot) {
        if (slot == 0) {
            return slot0Configs.kV;
        } else if (slot == 1) {
            return slot1Configs.kV;
        }
        return slot2Configs.kV;
    }

    public double getV() {
        return getV(0);
    }

    public double getG(int slot) {
        if (slot == 0) {
            return slot0Configs.kG;
        } else if (slot == 1) {
            return slot1Configs.kG;
        }
        return slot2Configs.kG;
    }

    public double getG() {
        return getG(0);
    }

}
