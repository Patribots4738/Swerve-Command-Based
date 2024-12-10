package frc.robot.subsystems.test;

import org.littletonrobotics.junction.AutoLog;

public interface KrakenTestIO {
    
    @AutoLog
    class KrakenTestIOInputs {

        public boolean krakenConnected = false;
        public double positionRotations = 0.0;
        public double targetPositionRotations = 0.0;
        public double velocityRPM = 0.0;
        public double targetVelocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double targetPercent = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelcius = 0.0;

    }

    default void updateInputs(KrakenTestIOInputs inputs) {}

}
