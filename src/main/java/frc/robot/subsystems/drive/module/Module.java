package frc.robot.subsystems.drive.module;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.MK4cSwerveModuleConstants;

public class Module {

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;
    private final double chassisAngularOffset;

    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
    private SwerveModuleState currentState = new SwerveModuleState(0.0, new Rotation2d());
    private SwerveModulePosition currentPosition = new SwerveModulePosition(0.0, new Rotation2d());

    public Module(ModuleIO io, int index, double chassisAngularOffset) {
        this.io = io;
        this.index = index;
        this.chassisAngularOffset = chassisAngularOffset;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Swerve/Module" + index, inputs);

        currentState.speedMetersPerSecond = inputs.driveVelocityMPS;
        currentState.angle = new Rotation2d(inputs.turnEncoderAbsPositionRads - chassisAngularOffset);
        currentPosition.distanceMeters = inputs.drivePositionMeters;
        currentPosition.angle = new Rotation2d(inputs.turnEncoderAbsPositionRads - chassisAngularOffset);
    }

    /**
     * Corrects the rotation2d and speed of the MK4c 
     * 
     * @param desiredState stored rotation 2d and speed 
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        if (!FieldConstants.IS_SIMULATION) {
            correctedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                    new Rotation2d(inputs.turnEncoderAbsPositionRads));
        }

        // Command driving and turning TalonFX towards their respective setpoints.
        io.runDriveVelocity(correctedDesiredState.speedMetersPerSecond);
        io.setTurnPosition(correctedDesiredState.angle.getRadians());

        this.desiredState = correctedDesiredState;
    }

    public void runDriveCharacterization(double input) {
        io.runDriveCharacterization(input);
    }

    public void runTurnCharacterization(double input) {
        io.runTurnCharacterization(input);
    }

    public void setBrakeMode(boolean brake) {
        io.setDriveBrakeMode(brake);
        io.setTurnBrakeMode(brake);
    }

    public void resetDriveEncoder() {
        io.resetDriveEncoder();
    }

    /**
     * Obtains current state of the MK4c module.
     * 
     * @return current module state
     */
    public SwerveModuleState getState() {
        return currentState;
    }

    /**
     * Obtains the desired state of MK4c the module.
     * 
     * @return desired module state
     */
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * Obtains the current position of the MK4c module.
     * 
     * @return current module position
     */
    public SwerveModulePosition getPosition() {
        return currentPosition;
    }

    /** 
     * Gets the rotations of the wheel converted to radians.
     */
    public double getDrivePositionRadians() {
        return inputs.drivePositionMeters * 2 * Math.PI / MK4cSwerveModuleConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    public boolean getDrivePositionFlipped() {
        return inputs.drivePositionFlipped;
    }

    public double getCharacterizationVelocity() {
        return inputs.driveVelocityMPS / MK4cSwerveModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR;
    }

}
