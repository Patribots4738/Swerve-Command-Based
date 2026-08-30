package frc.robot.subsystems.drive.module;

import frc.robot.util.Constants.MK5nSwerveModuleConstants;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

public class Module {

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;
    private final double chassisAngularOffset;

    private SwerveModuleVelocity desiredState = new SwerveModuleVelocity(0.0, new Rotation2d());
    private SwerveModuleVelocity currentState = new SwerveModuleVelocity(0.0, new Rotation2d());
    private SwerveModulePosition currentPosition = new SwerveModulePosition(0.0, new Rotation2d());

    public Module(ModuleIO io, int index, double chassisAngularOffset) {
        this.io = io;
        this.index = index;
        this.chassisAngularOffset = chassisAngularOffset;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Swerve/Module" + index, inputs);

        currentState.velocity = inputs.driveVelocityMPS;
        currentState.angle = new Rotation2d(inputs.turnEncoderAbsPositionRads - chassisAngularOffset);
        currentPosition.distance = inputs.drivePositionMeters;
        currentPosition.angle = new Rotation2d(inputs.turnEncoderAbsPositionRads - chassisAngularOffset);
    }

    /**
     * Corrects the rotation2d and speed of the MK5n 
     * 
     * @param desiredState stored rotation 2d and speed 
     */
    public void setDesiredState(SwerveModuleVelocity desiredState, double feedforward) {
        // Apply chassis angular offset to the desired state.
        this.desiredState.velocity *= desiredState.angle.minus(new Rotation2d(inputs.turnEncoderAbsPositionRads)).getCos();
        this.desiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        this.desiredState = this.desiredState.optimize(new Rotation2d(inputs.turnEncoderAbsPositionRads));

        // Command driving and turning TalonFX towards their respective setpoints.
        io.runDriveVelocity(this.desiredState.velocity, feedforward);
        io.setTurnPosition(this.desiredState.angle.getRadians());
    }

    public void setTurnZero() {
        io.setTurnPosition(0);
    }

    public void runDriveCharacterization(double input, double turnAngle) {
        io.runDriveCharacterization(input, turnAngle);
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
     * Obtains current state of the MK5n module.
     * 
     * @return current module state
     */
    public SwerveModuleVelocity getState() {
        return currentState;
    }

    /**
     * Obtains the desired state of MK5n the module.
     * 
     * @return desired module state
     */
    public SwerveModuleVelocity getDesiredState() {
        return desiredState;
    }

    /**
     * Obtains the current position of the MK5n module.
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
        return inputs.drivePositionMeters * 2 * Math.PI / MK5nSwerveModuleConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    public boolean getDrivePositionFlipped() {
        return inputs.drivePositionFlipped;
    }

    public double getDriveCharacterizationVelocity() {
        return inputs.driveVelocityMPS / MK5nSwerveModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR;
    }

    public double getTurnCharacterizationVelocity() {
        return inputs.turnInternalVelocityRadsPerSec / MK5nSwerveModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR;
    }

}
