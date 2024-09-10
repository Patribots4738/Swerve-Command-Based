package frc.robot.subsystems.drive.module;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {

    @AutoLog
    class ModuleIOInputs {

        public double drivePositionRads = 0.0; 
        public double driveVelocityRotationsPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveSupplyCurrentAmps = 0.0;


        public double turnPosition = 0.0;
        public double turnVelocityRotationsPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnSupplyCurrentAmps = 0.0;

        public SwerveModulePosition position = new SwerveModulePosition(); //
        public SwerveModuleState state = new SwerveModuleState();
    }

    default void updateInputs(ModuleIOInputs inputs) {
        
    }

}