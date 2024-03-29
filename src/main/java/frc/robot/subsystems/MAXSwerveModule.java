// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.Neo;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.ModuleConstants;

public class MAXSwerveModule {
    private final Neo drivingSpark;
    private final Neo turningSpark;

    private final RelativeEncoder drivingEncoder;
    private final AbsoluteEncoder turningEncoder;

    private final SparkPIDController drivingPIDController;
    private final SparkPIDController turningPIDController;

    private double chassisAngularOffset = 0;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        drivingSpark = new Neo(drivingCANId);
        turningSpark = new Neo(turningCANId);
        
        drivingEncoder = drivingSpark.getEncoder();
        turningEncoder = turningSpark.getAbsoluteEncoder(Type.kDutyCycle);
        drivingPIDController = drivingSpark.getPIDController();
        turningPIDController = turningSpark.getPIDController();
        
        configMotors();

        this.chassisAngularOffset = chassisAngularOffset;
        desiredState.angle = new Rotation2d(turningEncoder.getPosition());
        drivingEncoder.setPosition(0);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        if (FieldConstants.IS_SIMULATION) {
            return getSimState();
        }
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(drivingEncoder.getVelocity(),
                new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    public SwerveModuleState getSimState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(drivingSpark.getVelocity(),
                new Rotation2d(turningSpark.getPosition() - chassisAngularOffset));

    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        if (FieldConstants.IS_SIMULATION) {
            return getSimPosition();
        }
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
                drivingEncoder.getPosition(),
                new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    public SwerveModulePosition getSimPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
                drivingSpark.getPosition(),
                new Rotation2d(turningSpark.getPosition() - chassisAngularOffset));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                new Rotation2d(turningEncoder.getPosition()));

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        if (FieldConstants.IS_SIMULATION) {
            drivingSpark.setTargetVelocity(correctedDesiredState.speedMetersPerSecond);
            turningSpark.setTargetPosition(correctedDesiredState.angle.getRadians());
        }
        else {
            turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkBase.ControlType.kPosition);
            drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity);
        }

        this.desiredState = desiredState;
    }

    /**
     * Zeroes all the SwerveModule encoders.
     */
    public void resetEncoders() {
        drivingEncoder.setPosition(0);
    }

    /**
     * Set the motor to coast mode
     */
    public void setCoastMode() {
        drivingSpark.setIdleMode(CANSparkBase.IdleMode.kCoast);
        turningSpark.setIdleMode(CANSparkBase.IdleMode.kCoast);
    }

    /**
     * Set the motor to brake mode
     */
    public void setBrakeMode() {
        drivingSpark.setIdleMode(CANSparkBase.IdleMode.kBrake);
        turningSpark.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    public void tick() {
        drivingSpark.tick();
        turningSpark.tick();
    }

    public void configMotors() {
        turningSpark.restoreFactoryDefaults();
        drivingSpark.restoreFactoryDefaults();
        Timer.delay(0.25);
        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        drivingPIDController.setFeedbackDevice(drivingEncoder);
        turningPIDController.setFeedbackDevice(turningEncoder);

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        drivingEncoder.setPositionConversionFactor(ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR);
        drivingEncoder.setVelocityConversionFactor(ModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        turningEncoder.setPositionConversionFactor(ModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        turningEncoder.setInverted(ModuleConstants.TURNING_ENCODER_INVERTED);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        turningPIDController.setPositionPIDWrappingEnabled(true);
        turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT);
        turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT);

        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        drivingPIDController.setP(ModuleConstants.DRIVING_P);
        drivingPIDController.setI(ModuleConstants.DRIVING_I);
        drivingPIDController.setD(ModuleConstants.DRIVING_D);
        drivingPIDController.setFF(ModuleConstants.DRIVING_FF);
        drivingPIDController.setOutputRange(ModuleConstants.DRIVING_MIN_OUTPUT,
                ModuleConstants.DRIVING_MAX_OUTPUT);

        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        turningPIDController.setP(ModuleConstants.TURNING_P);
        turningPIDController.setI(ModuleConstants.TURNING_I);
        turningPIDController.setD(ModuleConstants.TURNING_D);
        turningPIDController.setFF(ModuleConstants.TURNING_FF);
        turningPIDController.setOutputRange(ModuleConstants.TURNING_MIN_OUTPUT,
                ModuleConstants.TURNING_MAX_OUTPUT);

        drivingSpark.setIdleMode(CANSparkBase.IdleMode.kBrake);
        turningSpark.setIdleMode(CANSparkBase.IdleMode.kBrake);
        drivingSpark.setSmartCurrentLimit(ModuleConstants.VORTEX_CURRENT_LIMIT);
        turningSpark.setSmartCurrentLimit(ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);
        
        // See https://docs.revrobotics.com/Spark/operating-modes/control-interfaces#periodic-status-5-default-rate-200ms
        drivingSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        turningSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        turningSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    }
}
