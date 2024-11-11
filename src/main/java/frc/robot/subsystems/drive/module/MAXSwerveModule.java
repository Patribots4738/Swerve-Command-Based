// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.module;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.MAXSwerveModuleConstants;
import frc.robot.util.hardware.rev.Neo;

public class MAXSwerveModule implements ModuleIO {
    private final Neo driveMotor;
    private final Neo turnMotor;

    private final int index;

    private double chassisAngularOffset = 0;

    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, int index) {
        driveMotor = new Neo(drivingCANId, true);
        
        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.

        turnMotor = new Neo(turningCANId, false, MAXSwerveModuleConstants.TURNING_ENCODER_INVERTED, true);
        this.chassisAngularOffset = chassisAngularOffset;
        this.index = index;
        resetEncoders();
        configMotors();
        desiredState.angle = new Rotation2d(turnMotor.getPosition());
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    @Override
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(inputs.driveVelocityMPS,
            new Rotation2d(inputs.turnInternalPositionRads - chassisAngularOffset));
    }

    public SparkPIDController getDrivingPIDController() {
        return driveMotor.getPIDController();
    }
    
    public SparkPIDController getTurningPIDController() {
        return turnMotor.getPIDController();
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    @Override
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
            inputs.drivePositionMeters,
            new Rotation2d(inputs.turnInternalPositionRads - chassisAngularOffset));
    }

    // gets the rotations of the wheel converted to radians
    // We need to undo the position conversion factor
    @Override
    public double getDrivePositionRadians() {
        return inputs.drivePositionMeters * 2 * Math.PI / MAXSwerveModuleConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        if (!FieldConstants.IS_SIMULATION) {
            correctedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                    new Rotation2d(inputs.turnInternalPositionRads));
        }

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        driveMotor.setTargetVelocity(correctedDesiredState.speedMetersPerSecond);
        turnMotor.setTargetPosition(correctedDesiredState.angle.getRadians());

        this.desiredState = correctedDesiredState;
    }

    @Override
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * Zeroes this driving motor's encoder.
     */
    @Override
    public void resetEncoders() {
        driveMotor.resetEncoder();
    }

    /**
     * Set the full module to coast mode
     */
    @Override
    public void setCoastMode() {
        driveMotor.setCoastMode();
        turnMotor.setCoastMode();
    }

    /**
     * Set the full module to brake mode
     */
    @Override
    public void setBrakeMode() {
        driveMotor.setBrakeMode();
        turnMotor.setBrakeMode();
    }

    public void configMotors() {
        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        driveMotor.setPositionConversionFactor(MAXSwerveModuleConstants.DRIVING_ENCODER_POSITION_FACTOR);
        driveMotor.setVelocityConversionFactor(MAXSwerveModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        turnMotor.setPositionConversionFactor(MAXSwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
        turnMotor.setVelocityConversionFactor(MAXSwerveModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        turnMotor.setPositionPIDWrappingEnabled(true);
        turnMotor.setPositionPIDWrappingBounds(
            MAXSwerveModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT,
            MAXSwerveModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT);

        driveMotor.setPID(MAXSwerveModuleConstants.DRIVING_PID);
        turnMotor.setPID(MAXSwerveModuleConstants.TURNING_PID);

        driveMotor.setSmartCurrentLimit(MAXSwerveModuleConstants.NEO_CURRENT_LIMIT);
        turnMotor.setSmartCurrentLimit(MAXSwerveModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);
        setBrakeMode();
    }
    
    @Override
    public void processInputs() {
        Logger.processInputs("SubsystemInputs/Swerve/MAXSwerveModule" + index, inputs);
    }
    
    @Override
    public void updateInputs() {
        // swerve module position and state

        // drive motor
        inputs.drivePositionMeters = driveMotor.getPosition();
        inputs.driveVelocityMPS = driveMotor.getVelocity();
        inputs.driveAppliedVolts = driveMotor.getAppliedOutput();
        inputs.driveSupplyCurrentAmps = driveMotor.getOutputCurrent();


        // turning motor
        inputs.turnAppliedVolts = turnMotor.getAppliedOutput();
        inputs.turnInternalPositionRads = turnMotor.getPosition();
        inputs.turnInternalVelocityRadsPerSec = turnMotor.getVelocity();
        inputs.turnSupplyCurrentAmps = turnMotor.getOutputCurrent();
        
    }

    // This issue does not happen with SparkMAX's! Hooray!
    @Override
    public boolean getDrivePositionFlipped() {
        return false;
    }
}