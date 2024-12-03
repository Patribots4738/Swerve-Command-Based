package frc.robot.util.hardware.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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

    private double positionConversionFactor = 1.0;
    private double velocityConversionFactor = 1.0;

    private final TalonFXConfigurator configurator = getConfigurator();
    private final TalonFXSimState sim = getSimState();

    private final SlotConfigs slotConfigs = new SlotConfigs();
    private final CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs();
    private final MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    private final ClosedLoopGeneralConfigs closedLoopConfigs = new ClosedLoopGeneralConfigs();
    private final FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    private final TorqueCurrentConfigs torqueCurrentConfigs = new TorqueCurrentConfigs();
    private final ClosedLoopRampsConfigs closedLoopRampConfigs = new ClosedLoopRampsConfigs();

    private DCMotorSim motorSimModel;

    private final PositionVoltage positionRequest;
    private final VelocityVoltage velocityRequest;
    private final PositionTorqueCurrentFOC positionTorqueRequest;
    private final VelocityTorqueCurrentFOC velocityTorqueRequest;
    private final VoltageOut voltageRequest;
    private final DutyCycleOut percentRequest;
    private final TorqueCurrentFOC torqueRequest;

    private final StatusSignal<Double> positionSignal;
    private final StatusSignal<Double> velocitySignal;
    private final StatusSignal<Double> voltageSignal;
    private final StatusSignal<Double> percentSignal;
    private final StatusSignal<Double> supplyCurrentSignal;
    private final StatusSignal<Double> statorCurrentSignal;   
    private final StatusSignal<Double> torqueCurrentSignal;
    private final StatusSignal<Double> temperatureSignal;

    private final GainConstants[] gains;

    private final boolean useFOC;
    private final boolean useTorqueControl;

    private TelemetryPreference telemetryPreference;

    /**
     * Creates new Kraken motor.
     * 
     * @param id ID of Kraken motor
     * @param canBus CANivore Kraken is connected to
     */
    public Kraken(int id, String canBus) {
        this(id, canBus, false, false, false);
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
        this(id, canBus, inverted, false, false);
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
    public Kraken(int id, boolean inverted, boolean useFOC, boolean useTorqueControl) {
        this(id, "rio", inverted, useFOC, useTorqueControl);
    }

    /**
     * Creates a new Kraken motor that can be inverted and use FOC.
     * 
     * @param id ID of Kraken motor
     * @param inverted inverts input given to motor when set to true
     * @param canBus CANivore Kraken is connected to
     * @param useFOC uses FOC to enhance motor communication when set to true
     */
    public Kraken(int id, String canBus, boolean inverted, boolean useFOC, boolean useTorqueControl) {

        super(id, canBus);

        this.useFOC = useFOC;
        this.useTorqueControl = useTorqueControl;

        positionRequest = new PositionVoltage(0).withEnableFOC(useFOC).withUpdateFreqHz(0);
        velocityRequest = new VelocityVoltage(0).withEnableFOC(useFOC).withUpdateFreqHz(0);
        positionTorqueRequest = new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);
        velocityTorqueRequest = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);
        voltageRequest = new VoltageOut(0).withEnableFOC(useFOC).withUpdateFreqHz(0);
        percentRequest = new DutyCycleOut(0).withEnableFOC(useFOC).withUpdateFreqHz(0);
        torqueRequest = new TorqueCurrentFOC(0).withUpdateFreqHz(0);

        gains = new GainConstants[] { new GainConstants(), new GainConstants(), new GainConstants() };

        positionSignal = super.getPosition();
        velocitySignal = super.getVelocity();
        voltageSignal = super.getMotorVoltage();
        percentSignal = super.getDutyCycle();
        supplyCurrentSignal = super.getSupplyCurrent();
        statorCurrentSignal = super.getStatorCurrent();
        torqueCurrentSignal = super.getTorqueCurrent();
        temperatureSignal = super.getDeviceTemp();

        setTelemetryPreference(TelemetryPreference.DEFAULT);
        optimizeBusUtilization(0, 1.0);

        setInverted(inverted);
        setBrakeMode();
        register();

    }

    public enum TelemetryPreference {
        DEFAULT,
        NO_ENCODER,
        SWERVE
    }

    /**
     * Changes the update frequency of certain status fields based on the preference
     * 
     * @param newPreference the new telemetry mode to switch to
     */
    public void setTelemetryPreference(TelemetryPreference newPreference) {
        telemetryPreference = newPreference;

        switch(telemetryPreference) {
            case NO_ENCODER:
                BaseStatusSignal.setUpdateFrequencyForAll(
                    KrakenMotorConstants.TALONFX_FAST_UPDATE_FREQ_HZ,
                    voltageSignal,
                    percentSignal
                );
                BaseStatusSignal.setUpdateFrequencyForAll(
                    KrakenMotorConstants.TALONFX_SLOW_UPDATE_FREQ_HZ, 
                    supplyCurrentSignal,
                    statorCurrentSignal,
                    torqueCurrentSignal,
                    temperatureSignal
                );
                BaseStatusSignal.setUpdateFrequencyForAll(
                    0,
                    positionSignal,
                    velocitySignal
                );
                break;
            case SWERVE:
                BaseStatusSignal.setUpdateFrequencyForAll(
                    KrakenMotorConstants.TALONFX_FAST_UPDATE_FREQ_HZ,
                    positionSignal,
                    velocitySignal,
                    voltageSignal,
                    supplyCurrentSignal,
                    torqueCurrentSignal
                );
                BaseStatusSignal.setUpdateFrequencyForAll(
                    KrakenMotorConstants.TALONFX_SLOW_UPDATE_FREQ_HZ, 
                    temperatureSignal
                );
                BaseStatusSignal.setUpdateFrequencyForAll(
                    0,
                    percentSignal,
                    statorCurrentSignal
                );
                break;
            default:
                BaseStatusSignal.setUpdateFrequencyForAll(
                    KrakenMotorConstants.TALONFX_FAST_UPDATE_FREQ_HZ, 
                    voltageSignal,
                    percentSignal
                );
                BaseStatusSignal.setUpdateFrequencyForAll(
                    KrakenMotorConstants.TALONFX_MID_UPDATE_FREQ_HZ,
                    positionSignal,
                    velocitySignal
                );
                BaseStatusSignal.setUpdateFrequencyForAll(
                    KrakenMotorConstants.TALONFX_SLOW_UPDATE_FREQ_HZ, 
                    supplyCurrentSignal,
                    statorCurrentSignal,
                    torqueCurrentSignal,
                    temperatureSignal
                );
                break;
        }
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
            useTorqueControl 
                ? positionTorqueRequest
                    .withPosition(position / positionConversionFactor)
                    .withFeedForward(feedForward)
                    .withSlot(slot)
                : positionRequest
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
            useTorqueControl 
                ? velocityTorqueRequest
                    .withVelocity(velocity / velocityConversionFactor)
                    .withFeedForward(feedForward)
                    .withSlot(slot)
                : velocityRequest
                    .withVelocity(velocity / velocityConversionFactor)
                    .withFeedForward(feedForward)
                    .withSlot(slot));
        if (velocity == 0) {
            setVoltageOutput(0.0);
        }
        targetVelocity = velocity;
    }

    /**
     * Directly sets the output of Kraken using percent of its maximum output.
     * 
     * @param percent should be between 1.0 to -1.0
     */
    public void setPercentOutput(double percent) {
        setControl(percentRequest.withOutput(percent));
        targetPercent = percent;
    }

    /**
     * Directly sets the output of the Kraken using volts
     * 
     * @param volts the voltage to set to, automatically capped at the supply voltage
     */
    public void setVoltageOutput(double volts) {
        setControl(voltageRequest.withOutput(volts));
    }

    /**
     * Directly sets the output of the Kraken using amps
     * 
     * @param amps the current to set to
     */
    public void setTorqueCurrentOutput(double amps) {
        setControl(torqueRequest.withOutput(amps));
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
        currentLimitConfigs.SupplyCurrentLimit = currentLimit;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;
        configurator.apply(currentLimitConfigs, 1.0);
    }

    /**
     * Sets the limit of the current supplied to the stator of the Kraken.
     * 
     * @param currentLimit maximum allowable current
     */
    public void setStatorCurrentLimit(double currentLimit) {
        currentLimitConfigs.StatorCurrentLimit = currentLimit;
        currentLimitConfigs.StatorCurrentLimitEnable = true;
        configurator.apply(currentLimitConfigs, 1.0);
    }   

    /**
     * Sets the limit of the current output when the kraken is being controlled via torque
     * 
     * @param reverseLimit miniumum allowable current
     * @param forwardLimit maximum allowable current
     */
    public void setTorqueCurrentLimits(double reverseLimit, double forwardLimit) {
        torqueCurrentConfigs.PeakReverseTorqueCurrent = reverseLimit;
        torqueCurrentConfigs.PeakForwardTorqueCurrent = forwardLimit;
        configurator.apply(torqueCurrentConfigs, 1.0);
    }

    /**
     * Sets the limit of the current output when the kraken is being controlled via torque
     * 
     * @param reverseLimit miniumum allowable current
     * @param forwardLimit maximum allowable current
     */
    public void setClosedLoopRampPeriod(double seconds) {
        if (useTorqueControl) {
            closedLoopRampConfigs.TorqueClosedLoopRampPeriod = seconds;
        } else {
            closedLoopRampConfigs.VoltageClosedLoopRampPeriod = seconds;
        }
        configurator.apply(closedLoopRampConfigs, 1.0);
    }

    /**
     * Resets encoder postion.
     * 
     * @param position position encoder should be set to
     */
    public void resetEncoder(double position) {
        setPosition(position / positionConversionFactor);
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
        outputConfigs.NeutralMode = NeutralModeValue.Brake;
        configurator.apply(outputConfigs, 1.0);
    }

    /**
     * Puts the Kraken in coast mode and the motor can spin freely.
     */
    public void setCoastMode() {
        outputConfigs.NeutralMode = NeutralModeValue.Coast;
        configurator.apply(outputConfigs, 1.0);
    }

    /**
     * Sets the ID of the Kraken's CANCoder.
     * 
     * @param canCoderId new CANCoder ID
     */
    public void setEncoder(int canCoderId, double mechanismReduction) {
        feedbackConfigs.FeedbackRemoteSensorID = canCoderId;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        feedbackConfigs.SensorToMechanismRatio = 1.0;
        feedbackConfigs.RotorToSensorRatio = mechanismReduction;
        configurator.apply(feedbackConfigs, 1.0);
    }

    /**
     * Enables PID wrapping.
     * 
     * @param enabled set to true if wrapping is enabled
     */
    public void setPositionClosedLoopWrappingEnabled(boolean enabled) {
        closedLoopConfigs.ContinuousWrap = enabled;
        configurator.apply(closedLoopConfigs, 1.0);
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
        return positionSignal.getValue() * positionConversionFactor;        
    }

    /**
     * Obtains the current velocity of the Kraken as a double.
     * 
     * @return current velocity
     */
    public double getVelocityAsDouble() {
        return velocitySignal.getValue() * velocityConversionFactor;
    }

    /**
     * Obtains the current voltage of the Kraken as a double.
     * 
     * @return current voltage
     */
    public double getVoltageAsDouble() {
        return voltageSignal.getValue();
    }

    /**
     * Obtains the current duty cycle output percentage of the Kraken as a double.
     * 
     * @return current percent
     */
    public double getPercentAsDouble() {
        return percentSignal.getValue();
    }

    /**
     * Represents current supplied to Kraken as a double.
     * 
     * @return supply of current to Kraken
     */
    public double getSupplyCurrentAsDouble() {
        return supplyCurrentSignal.getValue();
    }

    /**
     * Represents current supplied to the stator of the Kraken as a double.
     * 
     * @return supply of current to stator
     */
    public double getStatorCurrentAsDouble() {
        return statorCurrentSignal.getValue();
    }

    /**
     * Represents the current creating torque as a double.
     * 
     * @return torque current
     */
    public double getTorqueCurrentAsDouble() {
        return torqueCurrentSignal.getValue();
    }

    /**
     * Represents the current motor temperature in celcius as a double.
     * 
     * @return motor temp. in celcius
     */
    public double getTemperatureAsDouble() {
        return temperatureSignal.getValue();
    }

    /**
     * Based on the telemetry preference, check if every needed status signal 
     * is able to be successfully refreshed
     * 
     * @return whether the motor is connected or not
     */
    public boolean isConnected() {
        return 
            switch(telemetryPreference) {
                case NO_ENCODER -> 
                    BaseStatusSignal.refreshAll(
                        voltageSignal,
                        percentSignal,
                        supplyCurrentSignal,
                        statorCurrentSignal,
                        torqueCurrentSignal,
                        temperatureSignal
                    ).isOK();
                case SWERVE ->
                    BaseStatusSignal.refreshAll(
                        voltageSignal,
                        positionSignal,
                        velocitySignal,
                        supplyCurrentSignal,
                        torqueCurrentSignal,
                        temperatureSignal
                    ).isOK();
                default ->
                    BaseStatusSignal.refreshAll(
                        positionSignal,
                        velocitySignal,
                        voltageSignal,
                        percentSignal,
                        supplyCurrentSignal,
                        statorCurrentSignal,
                        torqueCurrentSignal,
                        temperatureSignal
                    ).isOK();
            };
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

    /**
     * Given a set of gains and a gain slot, applies them with the TalonFX's configurator
     * 
     * @param appliedGains the set of gains to apply
     * @param slot the gain slot to configure
     */
    private void applyGains(GainConstants appliedGains, int slot) {
        slotConfigs.SlotNumber = slot;
        slotConfigs.kP = appliedGains.getP();
        slotConfigs.kI = appliedGains.getI();
        slotConfigs.kD = appliedGains.getD();
        slotConfigs.kS = appliedGains.getS();
        slotConfigs.kV = appliedGains.getV();
        slotConfigs.kG = appliedGains.getG();
        configurator.apply(slotConfigs, 1.0);
    }

    /**
     * Configures the slot gains for a given slot
     * 
     * @param slot the gain slot to configure
     */
    private void applyGains(int slot) {
        GainConstants appliedGains = gains[slot];
        slotConfigs.SlotNumber = slot;
        slotConfigs.kP = appliedGains.getP();
        slotConfigs.kI = appliedGains.getI();
        slotConfigs.kD = appliedGains.getD();
        slotConfigs.kS = appliedGains.getS();
        slotConfigs.kV = appliedGains.getV();
        slotConfigs.kG = appliedGains.getG();
        configurator.apply(slotConfigs, 1.0);
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
        applyGains(gains[slot].withGains(P, I, D, S, V, G), slot);
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
     * Sets the gains for the specified slot.
     * 
     * @param constants the gains to set
     * @param slot the gain slot to configure
     */
    public void setGains(GainConstants constants, int slot) {
        gains[slot] = constants;
        applyGains(slot);
    }

    /**
     * Sets the gains for the specified slot.
     * 
     * @param constants the gains to set
     */
    public void setGains(GainConstants constants) {
        setGains(constants, 0);
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
        applyGains(gains[slot].withPID(P, I, D), slot);
    }

    /**
     * Sets the PID gains for the slot 0.
     * 
     * @param P proportional gains
     * @param I integral gains
     * @param D derivative gains
     */
    public void setPID(double P, double I, double D) {
        setPID(P, I, D, 0);
    }

    /**
     * Sets the proportional gain for the specified slot
     * 
     * @param P proportional gain
     * @param slot the gain slot to configure
     */
    public void setP(double P, int slot) {
        applyGains(gains[slot].withP(P), slot);
    }

    /**
     * Sets the proportional gain for slot 0
     * 
     * @param P proportional gain
     */
    public void setP(double P) {
        setP(P, 0);
    }

    /**
     * Sets the integral gain for the specified slot
     * 
     * @param I integral gain
     * @param slot the gain slot to configure
     */
    public void setI(double I, int slot) {
        applyGains(gains[slot].withI(I), slot);
    }

    /**
     * Sets the integral gain for slot 0
     * 
     * @param I integral gain
     */
    public void setI(double I) {
        setI(I, 0);
    }

    /**
     * Sets the derivative gain for the specified slot
     * 
     * @param D derivative gain
     * @param slot the gain slot to configure
     */
    public void setD(double D, int slot) {
        applyGains(gains[slot].withD(D), slot);
    }

    /**
     * Sets the derivative gain for slot 0
     * 
     * @param D derivative gain
     */
    public void setD(double D) {
        setD(D, 0);
    }

    /**
     * Sets the static feedforward gain for the specified slot
     * 
     * @param S static feedforward gain
     * @param slot the gain slot to configure
     */
    public void setS(double S, int slot) {
        applyGains(gains[slot].withS(S), slot);
    }

    /**
     * Sets the static feedforward gain for slot 0
     * 
     * @param S static feedforward gain
     */
    public void setS(double S) {
        setS(S, 0);
    }

    /**
     * Sets the velocity feedforward gain for the specified slot
     * 
     * @param V velocity feedforward gain
     * @param slot the gain slot to configure
     */
    public void setV(double V, int slot) {
        applyGains(gains[slot].withV(V), slot);
    }

    /**
     * Sets the velocity feedforward gain for slot 0
     * 
     * @param V velocity feedforward gain
     */
    public void setV(double V) {
        setV(V, 0);
    }

    /**
     * Sets the gravitational feedforward gain for the specified slot
     * 
     * @param G gravitational feedforward gain
     * @param slot the gain slot to configure
     */
    public void setG(double G, int slot) {
        applyGains(gains[slot].withG(G), slot);
    }

    /**
     * Sets the gravitational feedforward gain for slot 0
     * 
     * @param G gravitational feedforward gain
     */
    public void setG(double G) {
        setG(G, 0);
    }

    /**
     * Gets the proportional gain from the specified slot
     * 
     * @param slot the slot to get the gain from
     * @return the proportional gain
     */
    public double getP(int slot) {
        return gains[slot].getP();
    }

    /**
     * Gets the proportional gain from slot 0
     * 
     * @return the proportional gain
     */
    public double getP() {
        return getP(0);
    }

    /**
     * Gets the integral gain from the specified slot
     * 
     * @param slot the slot to get the gain from
     * @return the integral gain
     */
    public double getI(int slot) {
        return gains[slot].getI();
    }

    /**
     * Gets the integral gain from slot 0
     * 
     * @return the integral gain
     */
    public double getI() {
        return getI(0);
    }

    /**
     * Gets the derivative gain from the specified slot
     * 
     * @param slot the slot to get the gain from
     * @return the derivative gain
     */
    public double getD(int slot) {
        return gains[slot].getD();
    }

    /**
     * Gets the derivative gain from slot 0
     * 
     * @return the derivative gain
     */
    public double getD() {
        return getD(0);
    }

    /**
     * Gets the static feedforward gain from the specified slot
     * 
     * @param slot the slot to get the gain from
     * @return the static feedforward gain
     */
    public double getS(int slot) {
        return gains[slot].getS();
    }

    /**
     * Gets the static feedforward gain from slot 0
     * 
     * @return the static feedforward gain
     */
    public double getS() {
        return getS(0);
    }

    /**
     * Gets the velocity feedforward gain from the specified slot
     * 
     * @param slot the slot to get the gain from
     * @return the velocity feedforward gain
     */
    public double getV(int slot) {
        return gains[slot].getV();
    }

    /**
     * Gets the velocity feedforward gain from slot 0
     * 
     * @return the velocity feedforward gain
     */
    public double getV() {
        return getV(0);
    }

    /**
     * Gets the gravitational feedforward gain from the specified slot
     * 
     * @param slot the slot to get the gain from
     * @return the gravitational feedforward gain
     */
    public double getG(int slot) {
        return gains[slot].getG();
    }

    /**
     * Gets the gravitational feedforward gain from slot 0
     * 
     * @return the gravitational feedforward gain
     */
    public double getG() {
        return getG(0);
    }

}
