import frc.robot.util.Constants.RollerConstants;
import frc.robot.util.hardware.phoenix.Kraken.ControlPreference;

public class RollerIOKraken implements RollerIO {

    private final Kraken motor;

    public RollerIOKraken() {
        motor = new Kraken(RollerConstants.CAN_ID, true, true, ControlPreference.CURRENT);
        configMotor();
    }

    private void configMotor() {
        motor.setMotorInverted(RollerConstants.MOTOR_INVERTED);
        motor.setBrakeMode(RollerConstants.BRAKE_MOTOR);
        motor.setGearRatio(RollerConstants.GEAR_RATIO);
        motor.setSupplyCurrentLimit(RollerConstants.CURRENT_LIMIT);
        motor.setStatorCurrentLimit(RollerConstants.CURRENT_LIMIT);
    }

    @Override
    public void updateInputs(RollerIOKrakenInputs inputs) {
        inputs.motorConnected = motor.refreshSignals().isOK();
        inputs.percentOutput = motor.getPercentAsDouble();
        inputs.targetPercentOutput = motor.getTargetPercent();
        inputs.temperatureCelsius = motor.getTemperatureAsDouble();
        inputs.appliedOutputVolts = motor.getVoltageAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrentAsDouble();
    }


}











public class RollerIOKraken implements RollerIO {

    private final Kraken motor;

    public RollerIOKraken(int canId, int motorType, boolean useOneShot) {
        this.motor = new Kraken()

    }
}
