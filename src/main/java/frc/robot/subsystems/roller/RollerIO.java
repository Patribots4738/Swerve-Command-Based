import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

    @AutoLog
    class RollerIOInputs {
        public boolean motorConnected = false;
        public double percentOutput = 0.0;
        public double targetPercentOutput = 0.0;
        public double temperatureCelsius = 0.0;
        public double appliedOutputVolts = 0.0;
        public double torqueCurrentAmps = 0.0;
    }

    public default void updateInputs(RollerIOInputs inputs) {}

    public default void setPercent(double percent) {}

    public default void setBrakeMode(boolean brake) {}
    
    public default void setVoltage(double volts) {}

    public default void setCurrent(double amps) {}

    public default void setNeutral() {}
}
