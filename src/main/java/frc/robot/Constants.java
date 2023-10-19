// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

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
    /*
     * TAKE NOTE: This is a template file. It is intended to be copied and renamed
     * for each new robot and the constants adjusted as needed. It is not intended
     * to be used directly as the Constants class for a robot.
     */
    public static final class DriveConstants {
        public static final int LEFT_MOTOR1_CAN_ID = 0;
        public static final int LEFT_MOTOR2_CAN_ID = 1;
        public static final int RIGHT_MOTOR1_CAN_ID = 2;
        public static final int RIGHT_MOTOR2_CAN_ID = 3;

        // TODO: Tune this value
        public static final double TRACK_WIDTH_METERS = 0.69;
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
                TRACK_WIDTH_METERS);

        // TODO: Tune these values for your drive
        public static final double GEAR_RATIO = 5.95;
        public static final double WHEEL_DIAMETER_METERS = 0.15;
        public static final double ENCODER_POSITION_CONVERSION_FACTOR =
                // Assumes the encoders are directly mounted on the wheel shafts
                (WHEEL_DIAMETER_METERS * Math.PI) / GEAR_RATIO;

        /*
         * These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT! These
         * characterization values MUST be determined either experimentally or
         * theoretically for *your* robot's drive.
         */        
        //TODO: Tune this value
        public static final double ks_VOLTS = 0.22;
        public static final double kv_VOLT_SECONDS_PER_METER = 1.98;
        public static final double ka_VOLT_SECONDS_SQUARED_PER_METER = 0.2;

        // Example value only - as above, this must be tuned for your drive!
        //TODO: Tune this value
        public static final double kp_DRIVE_VEL = 8.5;
        
        //TODO: Tune these values
        private static final double kv_VOLT_SECONDS_PER_RADIAN = 1.5;
        private static final double ka_VOLT_SECONDS_SQUARED_PER_RADIAN = 0.3;
        public static final LinearSystem<N2, N2, N2> DRIVETRIAN_PLANT = 
            LinearSystemId.identifyDrivetrainSystem(
                kv_VOLT_SECONDS_PER_METER,
                ka_VOLT_SECONDS_SQUARED_PER_METER,
                kv_VOLT_SECONDS_PER_RADIAN,
                ka_VOLT_SECONDS_SQUARED_PER_RADIAN
            );
        //TODO: What should this be?
        public static final DCMotor GEARBOX = DCMotor.getNEO(1);
        public static final boolean ENCODER_REVERSED = false;
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        // Deadzone for the driver's controller
        //(must be tuned for the driver's controller and 
        // driving style)
        public static final double DRIVER_DEADZONE = 0.07;
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;
    }
}