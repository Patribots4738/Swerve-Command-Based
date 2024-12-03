package frc.robot.subsystems.drive.module;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.MK4cSwerveModuleConstants;
import frc.robot.util.hardware.phoenix.CANCoderCustom;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.hardware.phoenix.Kraken.TelemetryPreference;

public class MK4cSwerveModule implements ModuleIO {

    private final Kraken driveMotor;
    private final Kraken turnMotor;
    private final CANCoderCustom turnEncoder;

    private final int canCoderId;

    private final int index;

    private double chassisAngularOffset = 0;

    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    /**
     * Creates new MK4c swerve module.
     * 
     * @param drivingCANId CAN ID of the driving motor
     * @param turningCANId CAN ID of the turning motor
     * @param canCoderId CAN ID of the modules encoder
     * @param chassisAngularOffset angular offset of module to the chasis
     * @param index
     */
    public MK4cSwerveModule(int drivingCANId, int turningCANId, int canCoderId, double chassisAngularOffset, int index) {
        // TODO: CHANGE USETORQUECONTROL TO TRUE ONCE WE HAVE PHOENIX PRO
        driveMotor = new Kraken(drivingCANId, "rio", false, true, false);
        turnMotor = new Kraken(turningCANId, "rio", MK4cSwerveModuleConstants.INVERT_TURNING_MOTOR, true, false);
        turnEncoder = new CANCoderCustom(canCoderId, "rio");
        this.canCoderId = canCoderId;
        this.index = index;
        this.chassisAngularOffset = chassisAngularOffset;
        resetEncoders();
        configMotors();
        configEncoder();
        desiredState.angle = new Rotation2d(turnEncoder.getPosition().refresh().getValue());
    }

    /**
     * Updates the inputs sent to the Mk4c module.
     */
    @Override
    public void updateInputs() {

        // Call isConnected() to refresh all status signals
        inputs.driverMotorConnected = driveMotor.isConnected();
        // 800 m is roughly the value at which the talonfx position "flips", leading to odometry bugs
        inputs.drivePositionFlipped = (inputs.drivePositionMeters > 800 && driveMotor.getPositionAsDouble() < -800);
        inputs.drivePositionMeters = driveMotor.getPositionAsDouble();
        inputs.driveVelocityMPS = driveMotor.getVelocityAsDouble();
        inputs.driveAppliedVolts = driveMotor.getVoltageAsDouble();
        inputs.driveSupplyCurrentAmps = driveMotor.getSupplyCurrentAsDouble();
        inputs.driveStatorCurrentAmps = driveMotor.getStatorCurrentAsDouble();
        inputs.driveTempCelcius = driveMotor.getTemperatureAsDouble();
        
        // Call isConnected() to refresh all status signals
        inputs.turnMotorConnected = turnMotor.isConnected();
        inputs.turnInternalPositionRads = turnMotor.getPositionAsDouble();
        inputs.turnInternalVelocityRadsPerSec = turnMotor.getVelocityAsDouble();
        inputs.turnAppliedVolts = turnMotor.getVoltageAsDouble();
        inputs.turnSupplyCurrentAmps = turnMotor.getSupplyCurrentAsDouble();
        inputs.turnStatorCurrentAmps = turnMotor.getStatorCurrentAsDouble();
        inputs.turnTempCelcius = turnMotor.getTemperatureAsDouble();

        // Call isConnected() to refresh all status signals, but only if turn encoder is real
        if (!FieldConstants.IS_SIMULATION) {
            inputs.turnEncoderConnected = turnEncoder.isConnected();
        }

        inputs.turnEncoderAbsPositionRads = inputs.turnEncoderConnected ? turnEncoder.getAbsolutePositionAsDouble() : inputs.turnInternalPositionRads;
        inputs.turnEncoderPositionRads = inputs.turnEncoderConnected ? turnEncoder.getPositionAsDouble() : inputs.turnInternalPositionRads;
        inputs.turnEncoderVelocityRadsPerSec = inputs.turnEncoderConnected ? turnEncoder.getVelocityAsDouble() : inputs.turnInternalVelocityRadsPerSec;

    }

    /**
     * Logs the inputs sent to MK4c module.
     */
    @Override
    public void processInputs() {
        Logger.processInputs("SubsystemInputs/Swerve/MK4cSwerveModule" + index, inputs);
    }

    /**
     * Corrects the rotation2d and speed of the MK4c 
     * 
     * @param desiredState stored rotation 2d and speed 
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
                    new Rotation2d(inputs.turnEncoderAbsPositionRads));
        }

        // Command driving and turning TalonFX towards their respective setpoints.
        driveMotor.setTargetVelocity(correctedDesiredState.speedMetersPerSecond);
        turnMotor.setTargetPosition(correctedDesiredState.angle.getRadians());

        Logger.recordOutput("Subsystems/Swerve/Module" + index + "TargetVelocity", correctedDesiredState.speedMetersPerSecond);

        this.desiredState = correctedDesiredState;
    }

    /**
     * Resets MK4c module encoders to 0.
     */
    @Override
    public void resetEncoders()  {
        driveMotor.resetEncoder(800);
    }

    /**
     * Sets the MK4c module to coast mode and the motors can spin freely.
     */
    @Override
    public void setCoastMode() {
        driveMotor.setCoastMode();
        turnMotor.setCoastMode();
    }

    /**
     * Sets the MK4c module to brake mode.
     */
    @Override
    public void setBrakeMode() {
        driveMotor.setBrakeMode();
        turnMotor.setBrakeMode();
    }

    /**
     * Configures MK4c module's encoders, conversion factor, PID, and current limit.
     */
    private void configMotors() {

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        driveMotor.setPositionConversionFactor(MK4cSwerveModuleConstants.DRIVING_ENCODER_POSITION_FACTOR);
        driveMotor.setVelocityConversionFactor(MK4cSwerveModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        turnMotor.setPositionConversionFactor(MK4cSwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
        turnMotor.setVelocityConversionFactor(MK4cSwerveModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);

        // Set status signal update frequencies, optimized for swerve
        driveMotor.setTelemetryPreference(TelemetryPreference.SWERVE);
        turnMotor.setTelemetryPreference(TelemetryPreference.SWERVE);

        // We only want to ask for the abs encoder in real life
        if (!FieldConstants.IS_SIMULATION) {
            turnMotor.setEncoder(this.canCoderId, MK4cSwerveModuleConstants.TURNING_MOTOR_REDUCTION);
        }

        turnMotor.setPositionClosedLoopWrappingEnabled(true);

        driveMotor.setGains(MK4cSwerveModuleConstants.DRIVING_GAINS);
        turnMotor.setGains(MK4cSwerveModuleConstants.TURNING_GAINS);

        driveMotor.setTorqueCurrentLimits(
            -MK4cSwerveModuleConstants.DRIVING_MOTOR_TORQUE_LIMIT_AMPS,
            MK4cSwerveModuleConstants.DRIVING_MOTOR_TORQUE_LIMIT_AMPS);
        turnMotor.setTorqueCurrentLimits(
            -MK4cSwerveModuleConstants.TURNING_MOTOR_TORQUE_LIMIT_AMPS,
            MK4cSwerveModuleConstants.TURNING_MOTOR_TORQUE_LIMIT_AMPS);

        setBrakeMode();
    }

    private void configEncoder() {
        turnEncoder.setPositionConversionFactor(MK4cSwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
        turnEncoder.setVelocityConversionFactor(MK4cSwerveModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);
    }

    @Override
    public void runDriveVolts(double input) {
        driveMotor.setVoltageOutput(input);
    }

    @Override
    public void runDriveAmps(double input) {
        driveMotor.setTorqueCurrentOutput(input);
    }

    /**
     * Obtains current state of the MK4c module.
     * 
     * @return current module state
     */
    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(inputs.driveVelocityMPS,
            new Rotation2d(inputs.turnEncoderAbsPositionRads - chassisAngularOffset));
    }

    /**
     * Obtains the desired state of MK4c the module.
     * 
     * @return desired module state
     */
    @Override
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * Obtains the current position of the MK4c module.
     * 
     * @return current module position
     */
    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.drivePositionMeters,
            new Rotation2d(inputs.turnEncoderAbsPositionRads - chassisAngularOffset));
    }

    /** 
     * Gets the rotations of the wheel converted to radians.
     */
    //We need to undo the position conversion factor
    @Override
    public double getDrivePositionRadians() {
        return inputs.drivePositionMeters * 2 * Math.PI / MK4cSwerveModuleConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    @Override
    public boolean getDrivePositionFlipped() {
        return inputs.drivePositionFlipped;
    }

    @Override
    public double getCharacterizationVelocity() {
        return inputs.driveVelocityMPS / MK4cSwerveModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR;
    }
    
}
