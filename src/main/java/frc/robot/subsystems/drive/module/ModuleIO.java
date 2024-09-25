package frc.robot.subsystems.drive.module;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {

    @AutoLog
    class ModuleIOInputs {

        public double drivePositionMeters = 0.0; 
        public double driveVelocityMPS = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveSupplyCurrentAmps = 0.0;
        public double driveStatorCurrentAmps = 0.0;

        public double turnPositionRads = 0.0;
        public double turnVelocityRadsPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnSupplyCurrentAmps = 0.0;
        public double turnStatorCurrentAmps = 0.0;

        public SwerveModulePosition position = new SwerveModulePosition();
        public SwerveModuleState state = new SwerveModuleState();
    }

    default void updateInputs() {}

    default void processInputs() {}

    default void setDesiredState(SwerveModuleState state) {}

    default void resetEncoders() {}

    default void setCoastMode() {}

    default void setBrakeMode() {}

    public SwerveModuleState getState();

    public SwerveModuleState getDesiredState();

    public SwerveModulePosition getPosition();

    public double getDrivePositionRadians();

}