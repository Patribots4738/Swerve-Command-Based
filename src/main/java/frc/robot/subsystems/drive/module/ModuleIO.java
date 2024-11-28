package frc.robot.subsystems.drive.module;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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

    default void updateInputs() {}

    default void processInputs() {}

    default void setDesiredState(SwerveModuleState state) {}

    default void resetEncoders() {}

    default void setCoastMode() {}

    default void setBrakeMode() {}

    default void runDriveVolts(double input) {}

    default void runDriveAmps(double input) {}

    public boolean getDrivePositionFlipped();

    public SwerveModuleState getState();

    public SwerveModuleState getDesiredState();

    public SwerveModulePosition getPosition();

    public double getDrivePositionRadians();

    public double getCharacterizationVelocity();

}