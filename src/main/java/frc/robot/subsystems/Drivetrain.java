// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    // Robot swerve modules
    private final SwerveModule frontLeft;
    private final SwerveModule rearLeft;
    private final SwerveModule frontRight;
    private final SwerveModule rearRight;
    // The gyro sensor
    private final ADIS16470_IMU gyro;
    // Odometry class for tracking robot pose
    SwerveDriveOdometry odometry;

    private final List<SwerveModule> modules;

    private final SwerveModulePosition[] currentPositions;

    /** Creates a new DriveSubsystem. */
    public Drivetrain() {        
        frontLeft = new SwerveModule(
                DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
                DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
                DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

        frontRight = new SwerveModule(
                DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
                DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
                DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

        rearLeft = new SwerveModule(
                DriveConstants.REAR_LEFT_DRIVING_CAN_ID,
                DriveConstants.REAR_LEFT_TURNING_CAN_ID,
                DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

        rearRight = new SwerveModule(
                DriveConstants.REAR_RIGHT_DRIVING_CAN_ID,
                DriveConstants.REAR_RIGHT_TURNING_CAN_ID,
                DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);
                
        gyro = new ADIS16470_IMU();
        
        modules = List.of(
            frontLeft, 
            frontRight, 
            rearLeft, 
            rearRight
        );

        currentPositions = new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };

        updateModulePositions();

        odometry = new SwerveDriveOdometry(
                DriveConstants.DRIVE_KINEMATICS,
                getRotation2d(),
                currentPositions);

    }

    @Override
    public void periodic() {
        updateModuleStates();
        updateModulePositions();
        
        // Update the odometry in the periodic block
        odometry.update(
                getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        rearLeft.getPosition(),
                        rearRight.getPosition()
                });
    }

    private void updateModulePositions() {
        for (int i = 0; i < modules.size(); i++) {
            currentPositions[i] = modules.get(i).getPosition();
        }
    }

    private void updateModuleStates() {
        setModuleStates(
            new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                rearLeft.getState(),
                rearRight.getState()
            }
        );
    }

    public SwerveModulePosition[] getModulePositions(){
        return currentPositions;
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        rearLeft.getPosition(),
                        rearRight.getPosition()
                },
                pose);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                discretize(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        xSpeed, ySpeed, rot, getRotation2d())
                                : new ChassisSpeeds(xSpeed, ySpeed, rot),
                        DriveConstants.LOOP_PERIOD));

        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        rearLeft.setDesiredState(swerveModuleStates[2]);
        rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        modules.forEach(SwerveModule::resetEncoders);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
    }

    /**
     * Credit: WPIlib 2024
     * Discretizes a continuous-time chassis speed.
     *
     * @param vx    Forward velocity.
     * @param vy    Sideways velocity.
     * @param omega Angular velocity.
     * @param dt    The duration of the timestep the speeds should be applied for.
     */
    public static ChassisSpeeds discretize(double vx, double vy, double omega, double dt) {
        var desiredDeltaPose = new Pose2d(vx * dt, vy * dt, new Rotation2d(omega * dt));
        var twist = new Pose2d().log(desiredDeltaPose);
        return new ChassisSpeeds(twist.dx / dt, twist.dy / dt, twist.dtheta / dt);
    }

    /**
     * Discretizes a continuous-time chassis speed.
     *
     * @param continuousSpeeds The continuous speeds.
     * @param dt               The duration of the timestep the speeds should be
     *                         applied for.
     */
    public static ChassisSpeeds discretize(ChassisSpeeds continuousSpeeds, double dt) {
        return discretize(continuousSpeeds.vxMetersPerSecond, continuousSpeeds.vyMetersPerSecond,
                continuousSpeeds.omegaRadiansPerSecond, dt);
    }

    public void scheduleConfigCommands() {
        modules.forEach(SwerveModule::scheduleConfigCommands);
    }
}