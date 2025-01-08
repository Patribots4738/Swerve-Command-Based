package frc.robot.subsystems.drive.module;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.custom.GainConstants;

public interface ModuleIO {

    @AutoLog
    class ModuleIOInputs {

        public boolean driverMotorConnected = false;
        public boolean drivePositionFlipped = false;
        public double drivePositionMeters = 0.0; 
        public double driveVelocityMPS = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveSupplyCurrentAmps = 0.0;
        public double driveStatorCurrentAmps = 0.0;
        public double driveTempCelcius = 0.0;

        public boolean turnMotorConnected = false;
        public double turnInternalPositionRads = 0.0;
        public double turnInternalVelocityRadsPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnSupplyCurrentAmps = 0.0;
        public double turnStatorCurrentAmps = 0.0;
        public double turnTempCelcius = 0.0;

        public boolean turnEncoderConnected = false;
        public double turnEncoderPositionRads = 0.0;
        public double turnEncoderAbsPositionRads = 0.0;
        public double turnEncoderVelocityRadsPerSec = 0.0;
        
    }

    default void updateInputs(ModuleIOInputs inputs) {}

    default void resetDriveEncoder() {}

    default void setDriveBrakeMode(boolean brake) {}

    default void setTurnBrakeMode(boolean brake) {}

    default void runDriveCharacterization(double input, double turnAngle) {}

    default void runTurnCharacterization(double input) {}

    default void runDriveVelocity(double velocity) {}

    default void setTurnPosition(double position) {}

    default void setGains(GainConstants driveGains, GainConstants turnGains) {}

}