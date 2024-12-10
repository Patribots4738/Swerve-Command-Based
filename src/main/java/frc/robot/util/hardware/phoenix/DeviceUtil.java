package frc.robot.util.hardware.phoenix;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;

import frc.robot.util.Constants.KrakenMotorConstants;

public class DeviceUtil {
    
    /**
     * Applies a parameter to a device using the provided status code supplier.
     * 
     * @param statusCode The supplier that provides the status code.
     * @param name The name of the parameter.
     * @param deviceId The ID of the device.
     * @return The status code after applying the parameter.
     */
    public static StatusCode applyParameter(Supplier<StatusCode> statusCode, String name, int deviceId) {
        StatusCode status = statusCode.get();
        for (int i = 1; i <= (KrakenMotorConstants.SAFE_TALON_MODE ? 20 : 1); i++) {
            status = statusCode.get();
            if (status.isOK()) {
                System.err.println("Successfully applied " + name + " to device " + deviceId + " on attempt " + i);
                return status;
            } else {
                System.err.println("Failed to apply " + name + " to device " + deviceId + " on attempt " + i);
            }
        }
        return status;
    }

    /**
     * Applies the specified signal frequency to the given device and signals.
     * 
     * @param frequency The signal frequency to apply.
     * @param deviceId The ID of the device.
     * @param signals The signals to apply the frequency to.
     * @return The status code indicating the result of the operation.
     */
    public static StatusCode applySignalFrequency(double frequency, int deviceId, BaseStatusSignal... signals) {
        return DeviceUtil.applyParameter(
            () -> BaseStatusSignal.setUpdateFrequencyForAll(frequency, signals), 
            frequency + " Hz Signal Frequency",
            deviceId
        );
    }
}
