package frc.robot.util.hardware.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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

    public CANCoderCustom(int id, String canBus, boolean inverted, double offset) {
        super(id, canBus);

        configureMagnetSensor(inverted, offset);

        position = getPosition();
        absolutePosition = getAbsolutePosition();
        velocity = getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
            CANCoderConstants.ENCODER_UPDATE_FREQ_HZ,
            absolutePosition,
            position,
            velocity
        );

        optimizeBusUtilization(0, 1.0);

    }

    public boolean isConnected() {
        return BaseStatusSignal.refreshAll(absolutePosition, position, velocity).isOK();
    }

    public double getPositionAsDouble() {
        return position.getValue() * positionConversionFactor;
    }

    public double getAbsolutePositionAsDouble() {
        return absolutePosition.getValue() * positionConversionFactor;
    }

    public double getVelocityAsDouble() {
        return velocity.getValue() * velocityConversionFactor;
    }

    public void resetEncoder(double position) {
        setPosition(position / positionConversionFactor);
    }

    public void resetEncoder() {
        resetEncoder(0);
    }

    public void configureMagnetSensor(boolean inverted, double offset) {
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        magnetSensorConfigs.MagnetOffset = offset;
        magnetSensorConfigs.SensorDirection = inverted 
            ? SensorDirectionValue.CounterClockwise_Positive 
            : SensorDirectionValue.Clockwise_Positive;
        configurator.apply(magnetSensorConfigs, 1.0);
    }

    public void setPositionConversionFactor(double newFactor) {
        positionConversionFactor = newFactor;
    }

    public void setVelocityConversionFactor(double newFactor) {
        velocityConversionFactor = newFactor;
    }

}
