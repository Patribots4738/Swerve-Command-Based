package frc.robot.util.hardware.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Constants.PigeonConstants;

public class Pigeon2Custom extends Pigeon2 {

    private TelemetryPreference preference;

    private final StatusSignal<Double> yawSignal;
    private final StatusSignal<Double> pitchSignal;
    private final StatusSignal<Double> rollSignal;
    private final StatusSignal<Double> yawVelocitySignal;
    private final StatusSignal<Double> pitchVelocitySignal;
    private final StatusSignal<Double> rollVelocitySignal;

    public enum TelemetryPreference {
        DEFAULT,
        YAW_ONLY
    }

    public Pigeon2Custom(int id) {
        this(id, "rio");
    }
    
    public Pigeon2Custom(int id, String canBus) {
        super(id, canBus);
        yawSignal = getYaw();
        pitchSignal = getPitch();
        rollSignal = getRoll();
        yawVelocitySignal = getAngularVelocityZWorld();
        pitchVelocitySignal = getAngularVelocityYWorld();
        rollVelocitySignal = getAngularVelocityXWorld();

        setTelemetryPreference(TelemetryPreference.DEFAULT);
        optimizeBusUtilization(0, 1.0);
    }

    public void setTelemetryPreference(TelemetryPreference newPreference) {
        preference = newPreference;
        switch (preference) {
            case YAW_ONLY:
                BaseStatusSignal.setUpdateFrequencyForAll(
                    PigeonConstants.PIGEON_FAST_UPDATE_FREQ_HZ, 
                    yawSignal);
                BaseStatusSignal.setUpdateFrequencyForAll(
                    0,
                    pitchSignal,
                    rollSignal);
                break;
            default:
                BaseStatusSignal.setUpdateFrequencyForAll(
                    PigeonConstants.PIGEON_FAST_UPDATE_FREQ_HZ, 
                    yawSignal,
                    pitchSignal,
                    rollSignal);  
                break;
        }
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

    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getYawDegrees());
    }

    public boolean isConnected() {
        return 
            switch(preference) {
                case YAW_ONLY ->
                    BaseStatusSignal.refreshAll(
                        yawSignal,
                        yawVelocitySignal
                    ).isOK();
                default ->
                    BaseStatusSignal.refreshAll(
                        yawSignal,
                        pitchSignal,
                        rollSignal,
                        yawVelocitySignal,
                        pitchVelocitySignal,
                        rollVelocitySignal
                    ).isOK();
            };
    }

}
