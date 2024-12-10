package frc.robot.util.hardware.phoenix;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Constants.PigeonConstants;

public class Pigeon2Custom extends Pigeon2 {

    private TelemetryPreference telemetryPreference;

    private final StatusSignal<Double> yawSignal;
    private final StatusSignal<Double> pitchSignal;
    private final StatusSignal<Double> rollSignal;
    private final StatusSignal<Double> yawVelocitySignal;
    private final StatusSignal<Double> pitchVelocitySignal;
    private final StatusSignal<Double> rollVelocitySignal;

    public enum TelemetryPreference {
        DEFAULT,
        YAW_ONLY // only if we're feeling silly
    }

    public Pigeon2Custom(int id) {
        this(id, "rio");
    }
    
    /**
     * Represents a custom Pigeon2 object with additional functionality.
     *
     * @param id The ID of the Pigeon2 object.
     * @param canBus The CAN bus address of the Pigeon2 object.
     */
    public Pigeon2Custom(int id, String canBus) {
        super(id, canBus);
        yawSignal = getYaw();
        pitchSignal = getPitch();
        rollSignal = getRoll();
        yawVelocitySignal = getAngularVelocityZWorld();
        pitchVelocitySignal = getAngularVelocityYWorld();
        rollVelocitySignal = getAngularVelocityXWorld();

        setTelemetryPreference(TelemetryPreference.DEFAULT);
        applyParameter(
            () -> optimizeBusUtilization(0, 1.0),
            "Optimize Bus Utilization"
        );
    }

    /**
     * Applies a parameter to the Pigeon2Custom device.
     * 
     * @param configApplication a supplier that provides the configuration application
     * @param configName the name of the configuration
     * @return the status code indicating the result of applying the parameter
     */
    public StatusCode applyParameter(Supplier<StatusCode> configApplication, String configName) {
        return DeviceUtil.applyParameter(configApplication, configName, getDeviceID());
    }

    /**
     * Applies the specified signal frequency to the given status signals for the Pigeon2Custom device.
     *
     * @param frequency The signal frequency to be applied.
     * @param signals The status signals to which the frequency should be applied.
     * @return The status code indicating the success or failure of applying the signal frequency.
     */
    public StatusCode applySignalFrequency(double frequency, BaseStatusSignal... signals) {
        return DeviceUtil.applySignalFrequency(frequency, getDeviceID(), signals);
    }

    /**
     * Sets the telemetry preference for the Pigeon2Custom object.
     * 
     * @param newPreference the new telemetry preference to be set
     * @return true if the telemetry preference was set successfully, false otherwise
     */
    public boolean setTelemetryPreference(TelemetryPreference newPreference) {
        telemetryPreference = newPreference;

        return 
            switch (telemetryPreference) {
                case YAW_ONLY ->
                    applySignalFrequency(
                        PigeonConstants.PIGEON_FAST_UPDATE_FREQ_HZ, 
                        yawSignal,
                        yawVelocitySignal
                    ).isOK() &&
                    applySignalFrequency(
                        0,
                        pitchSignal,
                        pitchVelocitySignal,
                        rollSignal,
                        rollVelocitySignal
                    ).isOK();
                default ->
                    applySignalFrequency(
                        PigeonConstants.PIGEON_FAST_UPDATE_FREQ_HZ, 
                        yawSignal,
                        yawVelocitySignal,
                        pitchSignal,
                        pitchVelocitySignal,
                        rollSignal,
                        rollVelocitySignal
                    ).isOK();
            };
    }

    public double getYawDegrees() {
        return yawSignal.getValueAsDouble();
    }

    public double getYawRadians() {
        return Units.degreesToRadians(getYawDegrees());
    }

    public double getPitchDegrees() {
        return pitchSignal.getValueAsDouble();
    }

    public double getPitchRadians() {
        return Units.degreesToRadians(getPitchDegrees());
    }

    public double getRollDegrees() {
        return rollSignal.getValueAsDouble();
    }

    public double getRollRadians() {
        return Units.degreesToRadians(getRollDegrees());
    }

    public double getYawVelocityDegreesPerSec() {
        return yawVelocitySignal.getValueAsDouble();
    }

    public double getYawVelocityRadiansPerSec() {
        return Units.degreesToRadians(getYawVelocityDegreesPerSec());
    }

    public double getPitchVelocityDegreesPerSec() {
        return pitchVelocitySignal.getValueAsDouble();
    }

    public double getPitchVelocityRadiansPerSec() {
        return Units.degreesToRadians(getPitchVelocityDegreesPerSec());
    }

    public double getRollVelocityDegreesPerSec() {
        return rollVelocitySignal.getValueAsDouble();
    }

    public double getRollVelocityRadiansPerSec() {
        return Units.degreesToRadians(getRollVelocityDegreesPerSec());
    }

    public Rotation2d getYawRotation2d() {
        return Rotation2d.fromDegrees(getYawDegrees());
    }

    public Rotation2d getPitchRotation2d() {
        return Rotation2d.fromDegrees(getPitchDegrees());
    }

    public Rotation2d getRollRotation2d() {
        return Rotation2d.fromDegrees(getRollDegrees());
    }

    /**
     * Refreshes the signals based on the telemetry preference.
     * 
     * @return The status code indicating the success or failure of the signal refresh.
     */
    public StatusCode refreshSignals() {
        return 
            switch(telemetryPreference) {
                case YAW_ONLY ->
                    BaseStatusSignal.refreshAll(
                        yawSignal,
                        yawVelocitySignal
                    );
                default ->
                    BaseStatusSignal.refreshAll(
                        yawSignal,
                        pitchSignal,
                        rollSignal,
                        yawVelocitySignal,
                        pitchVelocitySignal,
                        rollVelocitySignal
                    );
            };
    }

}
