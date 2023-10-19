// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathRamsete;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    /*
     * This is were you define all the components of the subsystem. For example, if
     * your robot had two motors on the left side and two motors on the right side,
     * you would define them here.
     */

    // The motors on the drive.
    private final CANSparkMax leftMotor1;
    private final CANSparkMax leftMotor2;
    private final CANSparkMax rightMotor1;
    private final CANSparkMax rightMotor2;
    // The motors on the left side of the drive.
    private final MotorControllerGroup leftMotors;
    // The motors on the right side of the drive.
    private final MotorControllerGroup rightMotors;
    // The robot's drive
    private final DifferentialDrive drivetrain;
    // The left-side drive encoder
    private final RelativeEncoder leftEncoder;
    // The right-side drive encoder
    private final RelativeEncoder rightEncoder;
    // The gyro sensor
    private final ADIS16470_IMU gyro;
    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry odometry;
    // Things for sim stuff
    public DifferentialDrivetrainSim drivetrainSimulator;
    private final Field2d simField;
    private final EncoderSim leftEncoderSim;
    private final EncoderSim rightEncoderSim;
    private final ADIS16470_IMUSim gyroSim;


    /** Creates a new DriveSubsystem. */
    public Drivetrain() {
        /*
         * This is were you would initialize the motors etc... and (if needed)
         * invert them. For example, if your robot had two motors on the left side and
         * two motors on the right side, you would need to reverse one side so that
         * positive makes them both go forward.
         */
        // Initialize motors
        leftMotor1 = new CANSparkMax(DriveConstants.LEFT_MOTOR1_CAN_ID, MotorType.kBrushless);
        leftMotor2 = new CANSparkMax(DriveConstants.LEFT_MOTOR2_CAN_ID, MotorType.kBrushless);
        rightMotor1 = new CANSparkMax(DriveConstants.RIGHT_MOTOR1_CAN_ID, MotorType.kBrushless);
        rightMotor2 = new CANSparkMax(DriveConstants.RIGHT_MOTOR2_CAN_ID, MotorType.kBrushless);
        // Initialize motor groups
        leftMotors = new MotorControllerGroup(
            leftMotor1,
            leftMotor2);
        rightMotors = new MotorControllerGroup(
            rightMotor1,
            rightMotor2);
        // Initialize drivetrain
        drivetrain = new DifferentialDrive(leftMotors, rightMotors);
        // Initialize encoders
        leftEncoder = leftMotor1.getEncoder();
        rightEncoder = rightMotor1.getEncoder();
        // Initialize gyro
        gyro = new ADIS16470_IMU();
        // Initialize odometry
        odometry = new DifferentialDriveOdometry(
                getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

        AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getWheelSpeeds, // Current ChassisSpeeds supplier
            this::tankDriveChassisSpeeds, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            this // Reference to this subsystem to set requirements
        );

        if (RobotBase.isSimulation()) {
            drivetrainSimulator = new DifferentialDrivetrainSim(
                DriveConstants.DRIVETRIAN_PLANT,
                DriveConstants.GEARBOX,
                DriveConstants.GEAR_RATIO,
                DriveConstants.TRACK_WIDTH_METERS,
                (DriveConstants.WHEEL_DIAMETER_METERS / 2.0),
                VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005)
            );

            leftEncoderSim = new EncoderSim(new Encoder(
                DriveConstants.LEFT_MOTOR1_CAN_ID, 
                DriveConstants.LEFT_MOTOR2_CAN_ID,
                DriveConstants.ENCODER_REVERSED));

            rightEncoderSim = new EncoderSim(new Encoder(
                DriveConstants.RIGHT_MOTOR1_CAN_ID, 
                DriveConstants.RIGHT_MOTOR2_CAN_ID,
                DriveConstants.ENCODER_REVERSED));

            gyroSim = new ADIS16470_IMUSim(gyro);

            simField = new Field2d();
            SmartDashboard.putData("Field", simField);
        } else {
            drivetrainSimulator = null;
            leftEncoderSim = null;
            rightEncoderSim = null;
            gyroSim = null;
            simField = null;
        }

        /*
         * This is where the config commands are scheduled.
         * This is done different places in different codebases.
         *  
         * <p>
         * This is where you would configure the motors, set their inversion,
         * set the sensors to their starting states, etc...
         */
        scheduleConfigCommands();
    }

    /*
     * This is the periodic method. This is where you would put code that you want
     * to run periodically, such as reading sensors and updating the motors.
     * 
     * <p>
     * This method is called every 20ms when the robot is enabled.
     * The odometry is the robot's position on the field.
     */
    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update(
                getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }

    /**
     * This is a substitute for the method above
     */
    @Override
    public void simulationPeriodic() {
        drivetrainSimulator.setInputs(
            leftMotors.get() * RobotController.getBatteryVoltage(),
            rightMotors.get() * RobotController.getBatteryVoltage());

        drivetrainSimulator.update(0.02);

        leftEncoderSim.setDistance(drivetrainSimulator.getLeftPositionMeters());
        leftEncoderSim.setRate(drivetrainSimulator.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(drivetrainSimulator.getRightPositionMeters());
        rightEncoderSim.setRate(drivetrainSimulator.getRightVelocityMetersPerSecond());
        gyroSim.setGyroAngleY(-drivetrainSimulator.getHeading().getDegrees());
    }

    /**
     * @return The current draw of the drivetrain in amps.
     */
    public double getDrawnCurrentAmps(){
        return drivetrainSimulator.getCurrentDrawAmps();
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
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public ChassisSpeeds getWheelSpeeds() {
        return new ChassisSpeeds(
            leftEncoder.getVelocity(), 
            rightEncoder.getVelocity(), 
            getTurnRate());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        drivetrainSimulator.setPose(pose);
        odometry.resetPosition(
                getRotation2d(), 
                leftEncoder.getPosition(), 
                rightEncoder.getPosition(), 
                pose);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void drive(double fwd, double rot) {
        drivetrain.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     * 
     * <p>
     * This method is mainly used for autonomous driving.
     * One example is in the PPRamseteCommand.
     * 
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(rightVolts);
        drivetrain.feed();
    }

    public void tankDriveChassisSpeeds(ChassisSpeeds speeds) {
        var wheelSpeeds = DriveConstants.DRIVE_KINEMATICS.toWheelSpeeds(speeds);
        leftMotors.set(wheelSpeeds.leftMetersPerSecond);
        rightMotors.set(wheelSpeeds.rightMetersPerSecond);
        drivetrain.feed();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public RelativeEncoder getLeftEncoder() {
        return leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public RelativeEncoder getRightEncoder() {
        return rightEncoder;
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        drivetrain.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     * 
     * @return a Rotation2d representing the robot's heading
     */
    public Rotation2d getRotation2d() {
        return new Rotation2d(gyro.getAngle());
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
        return -gyro.getRate();
    }

    public FollowPathRamsete followPathCommand(String pathName){
        var path = PathPlannerPath.fromPathFile(pathName);

        return new FollowPathRamsete(
            path,
            this::getPose,
            this::getWheelSpeeds,
            this::tankDriveChassisSpeeds,
            new ReplanningConfig(),
            this
        );
    }

    public FollowPathWithEvents followEventPathCommand(String pathName){
        var path = PathPlannerPath.fromPathFile(pathName);

        return new FollowPathWithEvents(
            new FollowPathRamsete(
                path,
                this::getPose,
                this::getWheelSpeeds,
                this::tankDriveChassisSpeeds,
                new ReplanningConfig(),
                this
            ), 
            path, 
            this::getPose);        
    }
    

    /**
     * Schedules the config commands to run. This should be run every time the robot
     * is disabled.
     */
    public void scheduleConfigCommands() {
        CommandScheduler.getInstance().schedule(
            // Restore factory defaults
            new InstantCommand(() -> {
                leftMotor1.restoreFactoryDefaults();
                leftMotor2.restoreFactoryDefaults();
                rightMotor1.restoreFactoryDefaults();
                rightMotor2.restoreFactoryDefaults();
        
                leftEncoder.setPositionConversionFactor(DriveConstants.ENCODER_POSITION_CONVERSION_FACTOR);
                rightEncoder.setPositionConversionFactor(DriveConstants.ENCODER_POSITION_CONVERSION_FACTOR);
            
                resetEncoders();
                
                leftMotors.setInverted(true);
                rightMotors.setInverted(false);
                }
            ).ignoringDisable(true).asProxy()
        );
    }

    /**
     * Resets the relative rotation encoders to currently read a position of 0.
     */
    public void resetRelativeRotationEncoders() {
        CommandScheduler.getInstance().schedule(
                new InstantCommand(() -> resetEncoders()).ignoringDisable(false).asProxy());
    }

}