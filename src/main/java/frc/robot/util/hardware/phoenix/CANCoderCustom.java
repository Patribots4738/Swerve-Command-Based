package frc.robot.util.hardware.phoenix;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.util.Constants.CANCoderConstants;

public class CANCoderCustom extends CANcoder {

    private final MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();

    private final CANcoderConfigurator configurator = getConfigurator();

    private final StatusSignal<Double> position;
    private final StatusSignal<Double> absolutePosition;
    private final StatusSignal<Double> velocity;

    private double positionConversionFactor = 1.0;
    private double velocityConversionFactor = 1.0;

    public CANCoderCustom(int id) {
        this(id, "rio", false, 0);
    }

    public CANCoderCustom(int id, String canBus) {
        this(id, canBus, false, 0);
    }

    public CANCoderCustom(int id, double offset) {
        this(id, "rio", false, offset);
    }

    public CANCoderCustom(int id, boolean inverted, double offset) {
        this(id, "rio", inverted, offset);
    }

    /**
     * Represents a custom CANCoder with additional functionality.
     * Extends the base CANCoder class.
     *
     * @param id The unique identifier for the CANCoder.
     * @param canBus The CAN bus to which the CANCoder is connected.
     * @param inverted Whether the sensor readings should be inverted.
     * @param offset The offset to be applied to the sensor readings.
     */
    public CANCoderCustom(int id, String canBus, boolean inverted, double offset) {
        super(id, canBus);

        restoreFactoryDefaults();

        configureMagnetSensor(inverted, offset);

        position = getPosition();
        absolutePosition = getAbsolutePosition();
        velocity = getVelocity();

        applySignalFrequency(
            CANCoderConstants.ENCODER_UPDATE_FREQ_HZ,
            absolutePosition,
            position,
            velocity
        );
        applyParameter(
            () -> optimizeBusUtilization(0, 1.0), 
            "Optimize Bus Utilization"
        );

    }

    /**
     * Refreshes the signals of the CANCoder and returns the status code.
     *
     * @return The status code indicating the success or failure of the signal refresh.
     */
    public StatusCode refreshSignals() {
        return BaseStatusSignal.refreshAll(absolutePosition, position, velocity);
    }


    /**
     * Returns the position as a double value.
     * 
     * @return the position as rotations * PCF
     */
    public double getPositionAsDouble() {
        return position.getValue() * positionConversionFactor;
    }

    /**
     * Returns the absolute position as a double value.
     *
     * @return the absolute position as rotation from -0.5 to 0.5 * PCF
     */
    public double getAbsolutePositionAsDouble() {
        return absolutePosition.getValue() * positionConversionFactor;
    }

    /**
     * Returns the velocity as a double value.
     * 
     * @return the velocity as rps * VCF
     */
    public double getVelocityAsDouble() {
        return velocity.getValue() * velocityConversionFactor;
    }

    /**
     * Resets the encoder to the specified position.
     * 
     * @param position the desired position to reset the encoder to, position / PCF = rotations
     * @return the status code indicating the success or failure of the operation
     */
    public StatusCode resetEncoder(double position) {
        return applyParameter(
            () -> setPosition(position / positionConversionFactor),
            "Encoder Reset"
        );
    }

    public StatusCode resetEncoder() {
        return resetEncoder(0);
    }

    /**
     * Applies a parameter to the CANCoder device and returns the status code.
     *
     * @param configApplication the supplier function that applies the parameter configuration
     * @param name the name of the parameter
     * @return the status code indicating the result of applying the parameter
     */
    public StatusCode applyParameter(Supplier<StatusCode> configApplication, String name) {
        return DeviceUtil.applyParameter(configApplication, name, getDeviceID());
    }

    /**
     * Applies the specified signal frequency to the CANCoder device for the given status signals.
     *
     * @param frequency The signal frequency to be applied.
     * @param signals   The status signals to apply the frequency to.
     * @return The status code indicating the success or failure of applying the signal frequency.
     */
    public StatusCode applySignalFrequency(double frequency, BaseStatusSignal... signals) {
        return DeviceUtil.applySignalFrequency(frequency, getDeviceID(), signals);
    }

    /**
     * Configures the magnet sensor of the CANCoderCustom.
     * 
     * @param inverted true if the sensor direction is inverted, false otherwise
     * @param offset the magnet offset value
     * @return the status code indicating the success or failure of the configuration
     */
    public StatusCode configureMagnetSensor(boolean inverted, double offset) {
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        magnetSensorConfigs.MagnetOffset = offset;
        magnetSensorConfigs.SensorDirection = inverted 
            ? SensorDirectionValue.Clockwise_Positive 
            : SensorDirectionValue.CounterClockwise_Positive;
        return applyParameter(
            () -> configurator.apply(magnetSensorConfigs, 1.0), 
            "Magnet Sensor Configs"
        );
    }

    /**
     * Restores the factory defaults of the CANCoder.
     * 
     * @return The status code indicating the result of the operation.
     */
    public StatusCode restoreFactoryDefaults() {
        return applyParameter(
            () -> configurator.apply(new CANcoderConfiguration(), 1.0),
            "Factory Defaults"
        );
    }

    /**
     * Sets the position conversion factor for the CANCoder.
     * This factor is used to convert the raw position value to a meaningful unit.
     *
     * @param newFactor the new position conversion factor to be set
     */
    public void setPositionConversionFactor(double newFactor) {
        positionConversionFactor = newFactor;
    }

    /**
     * Sets the velocity conversion factor for the CANCoder.
     * This factor is used to convert the raw velocity value from the CANCoder to a meaningful unit.
     *
     * @param newFactor the new velocity conversion factor to be set
     */
    public void setVelocityConversionFactor(double newFactor) {
        velocityConversionFactor = newFactor;
    }

}
