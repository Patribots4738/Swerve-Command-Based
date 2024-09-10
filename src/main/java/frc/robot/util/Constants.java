// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.util.custom.PatrIDConstants;
import frc.robot.util.motor.rev.Neo;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class LoggingConstants {

        private static RobotType robotType = RobotType.DEVBOT;

        public static RobotType getRobot() {
            if (!FieldConstants.IS_SIMULATION && robotType == RobotType.SIMBOT) {
                System.out.println("Incorrect robot type selected, changing to real robot");
                robotType = RobotType.COMPBOT;
            }
            return robotType;
        }

        public static Mode getMode() {
            return switch (getRobot()) {
                case DEVBOT -> FieldConstants.IS_SIMULATION ? Mode.SIM : Mode.REAL;
                case COMPBOT -> FieldConstants.IS_SIMULATION ? Mode.REPLAY : Mode.REAL;
                case SIMBOT -> Mode.SIM;
            };
        }

        public enum Mode {
            /** Running on a real robot. */
            REAL,
            /** Running a physics simulator. */
            SIM,
            /** Replaying from a log file. */
            REPLAY
        }

        public enum RobotType {
            DEVBOT,
            COMPBOT,
            SIMBOT
        }

    }

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static double MAX_SPEED_METERS_PER_SECOND = AutoConstants.MAX_SPEED_METERS_PER_SECOND;

        public static final double MAX_ANGULAR_SPEED_RADS_PER_SECOND = Units.degreesToRadians(1137.21); // radians per second

        public static final double MAX_TELEOP_SPEED_METERS_PER_SECOND = 4.7;

        public static final double PASS_ROTATION_DEADBAND = 10;

        public static final double ODOMETRY_FREQUENCY = 250.0;

        // Chassis configuration
        // Distance between centers of right and left wheels on robot
        public static final double TRACK_WIDTH = Units.inchesToMeters(25.5);
        // Distance between front and back wheels on robot
        // Easiest measured from the center of the bore of the vortex
        public static final double WHEEL_BASE = Units.inchesToMeters(25.5);

        public static final double ROBOT_LENGTH_METERS = Units.inchesToMeters(29);
        public static final double BUMPER_LENGTH_METERS = Units.inchesToMeters(2.75);

        // Front positive, left positive
        public static final Translation2d FRONT_LEFT_WHEEL_POSITION = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d FRONT_RIGHT_WHEEL_POSITION = new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
        public static final Translation2d REAR_LEFT_WHEEL_POSITION = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d REAR_RIGHT_WHEEL_POSITION = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);

        public static final Translation2d[] WHEEL_POSITION_ARRAY = new Translation2d[] {
            FRONT_LEFT_WHEEL_POSITION,
            FRONT_RIGHT_WHEEL_POSITION,
            REAR_LEFT_WHEEL_POSITION,
            REAR_RIGHT_WHEEL_POSITION
        };

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                WHEEL_POSITION_ARRAY
        );

        // Angular offsets of the modules relative to the chassis in radians
        // add 90 degrees to change the X and Y axis
        public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(180 + 90);
        public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(-90 + 90);
        public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(90 + 90);
        public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(0 + 90);

        // Driving motors CAN IDs (EVEN)
        public static final int FRONT_LEFT_DRIVING_CAN_ID = 3;
        public static final int REAR_LEFT_DRIVING_CAN_ID = 5;
        public static final int FRONT_RIGHT_DRIVING_CAN_ID = 1;
        public static final int REAR_RIGHT_DRIVING_CAN_ID = 7;

        // Turning motors CAN IDs (ODD)
        public static final int FRONT_LEFT_TURNING_CAN_ID = 4;
        public static final int REAR_LEFT_TURNING_CAN_ID = 6;
        public static final int FRONT_RIGHT_TURNING_CAN_ID = 2;
        public static final int REAR_RIGHT_TURNING_CAN_ID = 8;

        public static final int GYRO_CAN_ID = 9;
        public static final boolean GYRO_REVERSED = true;
    }

    public static final class AutoConstants {

        // The below values need to be tuned for each new robot.
        // They are currently set to the values suggested by Choreo
        public static final double MAX_SPEED_METERS_PER_SECOND = 5;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 5;
        // Below is gotten from choreo
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Units.degreesToRadians(1137.21);
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Units.degreesToRadians(792.90);

        public static final double AUTO_POSITION_TOLERANCE_METERS = Units.inchesToMeters(1);
        public static final double AUTO_ROTATION_TOLERANCE_RADIANS = Units.degreesToRadians(2);
        public static final double PASS_ROTATION_TOLERANCE_RADIANS = Units.degreesToRadians(10);

        public static final double AUTO_ALIGNMENT_DEADBAND = Units.inchesToMeters(3);

        /*
         * XY:
         *  P: 5.2
         *  I: 0.125
         *  D: 0.0125
         * 
         * Theta:
         *   P: 1.3325
         *   I: 1 (izone on 20 degrees)
         *   D: 0.0375
         */
        public static final double XY_CORRECTION_P = 4;
        public static final double XY_CORRECTION_I = 0.0125;
        public static final double XY_CORRECTION_D = 0.0125;

        private static final PIDController XY_PID = new PIDController(
                AutoConstants.XY_CORRECTION_P,
                0,
                AutoConstants.XY_CORRECTION_D);

        public static final double ROTATION_CORRECTION_P = 3.725;
        public static final double ROTATION_CORRECTION_I = 0;
        public static final double ROTATION_CORRECTION_D = 0;

        private static final ProfiledPIDController THETA_PID = new ProfiledPIDController(
            AutoConstants.ROTATION_CORRECTION_P,
            AutoConstants.ROTATION_CORRECTION_I,
            AutoConstants.ROTATION_CORRECTION_D,
            new TrapezoidProfile.Constraints(
                    AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                    AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED)) 
            {{
                setIZone(Units.degreesToRadians(45));
            }};

        // Constraint for the motion-profiled robot angle controller
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

        public static HolonomicDriveController HDC = new HolonomicDriveController(
                XY_PID,
                XY_PID,
                THETA_PID
            );

        public static HolonomicPathFollowerConfig HPFC = new HolonomicPathFollowerConfig(
            new PIDConstants(
                AutoConstants.XY_CORRECTION_P,
                AutoConstants.XY_CORRECTION_I,
                AutoConstants.XY_CORRECTION_D),
            new PIDConstants(
                    AutoConstants.ROTATION_CORRECTION_P,
                    AutoConstants.ROTATION_CORRECTION_I,
                    AutoConstants.ROTATION_CORRECTION_D,
                    Units.degreesToRadians(45)),
            MAX_SPEED_METERS_PER_SECOND,
            Math.hypot(DriveConstants.WHEEL_BASE, DriveConstants.TRACK_WIDTH)/2.0,
            new ReplanningConfig());

        public static final String[] AUTO_NAMES = new String[] {};

    }

    public static final class ModuleConstants {
        // https://www.revrobotics.com/rev-21-3005/
        private enum SwerveGearing {
            LOW         (12, 22, 4.12, 4.92),
            MEDIUM      (13, 22, 4.46, 5.33),
            HIGH        (14, 22, 4.8, 5.5/*5.74*/),

            EXTRA_HIGH_1(14, 21, 5.03, 6.01),
            EXTRA_HIGH_2(14, 20, 5.28, 6.32),
            EXTRA_HIGH_3(15, 20, 5.66, 6.77),
            EXTRA_HIGH_4(16, 20, 6.04, 7.22),
            EXTRA_HIGH_5(16, 19, 6.36, 7.60);

            private final double 
                pinionTeeth, 
                spurTeeth, 
                maxSpeedNeo,
                maxSpeedVortex;

            SwerveGearing(
                int pinionTeeth, 
                int spurTeeth, 
                double maxSpeedNeo,
                double maxSpeedVortex)
            {
                this.pinionTeeth = pinionTeeth;
                this.spurTeeth = spurTeeth;
                this.maxSpeedNeo = maxSpeedNeo;
                this.maxSpeedVortex = maxSpeedVortex;
            }
        }

        public static final SwerveGearing CURRENT_GEARING = SwerveGearing.HIGH;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean TURNING_ENCODER_INVERTED = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.VORTEX_FREE_SPEED_RPM / 60;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(1.4642497827983136*2.0);
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        // 45 teeth on the wheel's bevel gear, 15 teeth on the bevel pinion
        public static final double DRIVING_MOTOR_REDUCTION = (45.0 * CURRENT_GEARING.spurTeeth) / (CURRENT_GEARING.pinionTeeth * 15.0);
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS
                * WHEEL_CIRCUMFERENCE_METERS)
                / DRIVING_MOTOR_REDUCTION;

        public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_CIRCUMFERENCE_METERS)
                / DRIVING_MOTOR_REDUCTION; // meters
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR = (WHEEL_CIRCUMFERENCE_METERS
                / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

        public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
        public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

        public static final double DRIVING_P = 0.256;
        public static final double DRIVING_I = 0;
        public static final double DRIVING_D = 0.255;
        public static final double DRIVING_FF = 0.20217;
        public static final double DRIVING_MIN_OUTPUT = -1;
        public static final double DRIVING_MAX_OUTPUT = 1;
        public static final PatrIDConstants DRIVING_PID = new PatrIDConstants(
            DRIVING_P,
            DRIVING_I,
            DRIVING_D,
            DRIVING_FF,
            DRIVING_MIN_OUTPUT,
            DRIVING_MAX_OUTPUT
        );

        public static final double TURNING_P = Robot.isSimulation() ? 0.5 : 1.5;
        public static final double TURNING_I = 0;
        public static final double TURNING_D = 1;
        public static final double TURNING_FF = 0;
        public static final double TURNING_MIN_OUTPUT = -1;
        public static final double TURNING_MAX_OUTPUT = 1;
        public static final PatrIDConstants TURNING_PID = new PatrIDConstants(
            TURNING_P,
            TURNING_I,
            TURNING_D,
            TURNING_FF,
            TURNING_MIN_OUTPUT,
            TURNING_MAX_OUTPUT
        );

        public static final int NEO_CURRENT_LIMIT = 50; // amps
        public static final int VORTEX_CURRENT_LIMIT = 80; // amps
        public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps
    }

    public static final class OIConstants {

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static final double DRIVER_DEADBAND = 0.15;
        public static final double OPERATOR_DEADBAND = 0.15;

        // See https://www.desmos.com/calculator/e07raajzh5
        // And
        // https://docs.google.com/spreadsheets/d/1Lytrh6q9jkz4u1gmF1Sk8kTpj8DxW-uwRE_QMnTt8Lk
        public static final double CONTROLLER_CORNER_SLOPE_1 = 1 / 0.7;
        public static final double CONTROLLER_CORNER_SLOPE_2 = 0.7;
    }

    public static final class NeoMotorConstants {
        public static final double VORTEX_FREE_SPEED_RPM = 6784;

        public static ArrayList<CANSparkBase> motors = new ArrayList<>();

        public static final boolean SAFE_SPARK_MODE = false;
        public static final double NEO_FREE_SPEED_RPM = 5676;

        public static final int MAX_PERIODIC_STATUS_TIME_MS = 32766;
        public static final int FAST_PERIODIC_STATUS_TIME_MS = 15;
        // This gets filled out as motors are created on the robot
        public static final HashMap<Integer, Neo> MOTOR_MAP = new HashMap<Integer, Neo>();

        public static final HashMap<Integer, String> CAN_ID_MAP = new HashMap<Integer, String>() {{
            /*  1  */ put(DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID, "FrontRightDrive");
            /*  2  */ put(DriveConstants.FRONT_RIGHT_TURNING_CAN_ID, "FrontRightTurn");
            /*  3  */ put(DriveConstants.FRONT_LEFT_DRIVING_CAN_ID, "FrontLeftDrive");
            /*  4  */ put(DriveConstants.FRONT_LEFT_TURNING_CAN_ID, "FrontLeftTurn");
            /*  5  */ put(DriveConstants.REAR_LEFT_DRIVING_CAN_ID, "RearLeftDrive");
            /*  6  */ put(DriveConstants.REAR_LEFT_TURNING_CAN_ID, "RearLeftTurn");
            /*  7  */ put(DriveConstants.REAR_RIGHT_DRIVING_CAN_ID, "RearRightDrive");
            /*  8  */ put(DriveConstants.REAR_RIGHT_TURNING_CAN_ID, "RearRightTurn");
        }};

        public static final HashMap<String, List<Neo>> MOTOR_GROUPS = new HashMap<String, List<Neo>>();

        public static Map<String, List<Neo>> initializeMotorGroupMap() {
            MOTOR_GROUPS.put("Drive", new ArrayList<Neo>() {{
                add(MOTOR_MAP.get(DriveConstants.FRONT_LEFT_DRIVING_CAN_ID));
                add(MOTOR_MAP.get(DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID));
                add(MOTOR_MAP.get(DriveConstants.REAR_LEFT_DRIVING_CAN_ID));
                add(MOTOR_MAP.get(DriveConstants.REAR_RIGHT_DRIVING_CAN_ID));
            }});
            MOTOR_GROUPS.put("Turn", new ArrayList<Neo>() {{
                add(MOTOR_MAP.get(DriveConstants.FRONT_LEFT_TURNING_CAN_ID));
                add(MOTOR_MAP.get(DriveConstants.FRONT_RIGHT_TURNING_CAN_ID));
                add(MOTOR_MAP.get(DriveConstants.REAR_LEFT_TURNING_CAN_ID));
                add(MOTOR_MAP.get(DriveConstants.REAR_RIGHT_TURNING_CAN_ID));
            }});

            return MOTOR_GROUPS;
        }

    }

    public static final class IntakeConstants {
        public static final int INTAKE_CAN_ID = 10;
        public static final double INTAKE_SPEED = 0.5;
        public static final double OUTTAKE_SPEED = -0.5;
        public static final double STOP_SPEED = 0;
        public static final int INTAKE_FREE_LIMIT = 15;
        public static final int INTAKE_STALL_LIMIT = 7;
    }

    public static final class FieldConstants {

        public static boolean IS_SIMULATION = Robot.isSimulation();

        public static final double ALIGNMENT_SPEED = 3;
        public static final double SNAP_TO_ANGLE_P = 0.0025;

        public static final double CONE_OFFSET_METERS = 0.542615;
        public static final double GRID_BARRIER_METERS = Units.inchesToMeters(12); // real is 14-15
        public static final double SUBSTATION_OFFSET_METERS = 0.7;
        public static final double ALLOWABLE_ERROR_METERS = Units.inchesToMeters(2);
        public static final double FIELD_WIDTH_METERS = 16.53;

    }

}