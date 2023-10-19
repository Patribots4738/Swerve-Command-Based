// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int frontLeftDriveMotorPort = 0;
    public static final int rearLeftDriveMotorPort = 2;
    public static final int frontRightDriveMotorPort = 4;
    public static final int rearRightDriveMotorPort = 6;

    public static final int frontLeftTurningMotorPort = 1;
    public static final int rearLeftTurningMotorPort = 3;
    public static final int frontRightTurningMotorPort = 5;
    public static final int rearRightTurningMotorPort = 7;

    public static final int[] frontLeftTurningEncoderPorts = new int[] {0, 1};
    public static final int[] rearLeftTurningEncoderPorts = new int[] {2, 3};
    public static final int[] frontRightTurningEncoderPorts = new int[] {4, 5};
    public static final int[] rearRightTurningEncoderPorts = new int[] {6, 7};

    public static final boolean frontLeftTurningEncoderReversed = false;
    public static final boolean rearLeftTurningEncoderReversed = true;
    public static final boolean frontRightTurningEncoderReversed = false;
    public static final boolean rearRightTurningEncoderReversed = true;

    public static final int[] frontLeftDriveEncoderPorts = new int[] {8, 9};
    public static final int[] rearLeftDriveEncoderPorts = new int[] {10, 11};
    public static final int[] frontRightDriveEncoderPorts = new int[] {12, 13};
    public static final int[] rearRightDriveEncoderPorts = new int[] {14, 15};

    public static final boolean frontLeftDriveEncoderReversed = false;
    public static final boolean rearLeftDriveEncoderReversed = true;
    public static final boolean frontRightDriveEncoderReversed = false;
    public static final boolean rearRightDriveEncoderReversed = true;

    // If you call DriveSubsystem.drive() with a different period make sure to update this.
    public static final double drivePeriod = TimedRobot.kDefaultPeriod;

    public static final double tracwidth = 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double wheelBase = 0.7;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics driveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2, tracwidth / 2),
            new Translation2d(wheelBase / 2, -tracwidth / 2),
            new Translation2d(-wheelBase / 2, tracwidth / 2),
            new Translation2d(-wheelBase / 2, -tracwidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double maxSpeedMetersPerSecond = 3;
  }

  public static final class ModuleConstants {
    public static final double maxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double maxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int encoderCPR = 1024;
    public static final double wheelDiameterMeters = 0.15;
    public static final double driveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (wheelDiameterMeters * Math.PI) / (double) encoderCPR;

    public static final double turningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) encoderCPR;

    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = 1;
  }

  public static final class OIConstants {
    public static final int driverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double maxSpeedMetersPerSecond = 3;
    public static final double maxAccelerationMetersPerSecondSquared = 3;
    public static final double maxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double maxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints thetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            maxAngularSpeedRadiansPerSecond, maxAngularSpeedRadiansPerSecondSquared);
  }
}