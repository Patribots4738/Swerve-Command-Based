package frc.robot;

import java.util.Optional;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Swerve;
import frc.robot.util.PatriBoxController;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NeoMotorConstants;
import frc.robot.util.Constants.OIConstants;
import monologue.Logged;

public class RobotContainer implements Logged {

    private final PatriBoxController driver;
    @SuppressWarnings("unused")
    private final PatriBoxController operator;

    private final Swerve swerve;
    @SuppressWarnings("unused")
    private final DriverUI driverUI;
    
    public RobotContainer() {
        driver = new PatriBoxController(OIConstants.DRIVER_CONTROLLER_PORT, OIConstants.DRIVER_DEADBAND);
        operator = new PatriBoxController(OIConstants.OPERATOR_CONTROLLER_PORT, OIConstants.OPERATOR_DEADBAND);
        DriverStation.silenceJoystickConnectionWarning(true);

        swerve = new Swerve();
        driverUI = new DriverUI();

        swerve.setDefaultCommand(new Drive(
            swerve,
            driver::getLeftY,
            driver::getLeftX,
            () -> -driver.getRightX(),
            () -> !driver.y().getAsBoolean(),
            () -> !driver.y().getAsBoolean(),
            () -> (driver.y().getAsBoolean() && FieldConstants.ALLIANCE.equals(Optional.of(Alliance.Blue)))
        ));

        swerve.configurateMotors();

        incinerateMotors();
        configureButtonBindings();

        Commands.sequence(Commands.runOnce(DriverStation::refreshData).repeatedly()
                .until(DriverStation.getAlliance()::isPresent), update(),
                Commands.runOnce(() -> System.out
                        .println("\n\n\n\nAlliance: " + FieldConstants.ALLIANCE))).ignoringDisable(true).schedule();
    }

    private void configureButtonBindings(){
        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureOperatorBindings() { }

    private void configureDriverBindings() {

        driver.start().or(driver.back()).onTrue(
            Commands.runOnce(() -> swerve.resetOdometry(
                new Pose2d(
                    swerve.getPose().getTranslation(), 
                    Rotation2d.fromDegrees(
                        FieldConstants.ALLIANCE.equals(Optional.of(Alliance.Red)) 
                        ? 0 
                        : 180))
            ), swerve)
        );

        driver.leftBumper().whileTrue(Commands.run(swerve::getSetWheelsX));

        driver.leftStick().toggleOnTrue(swerve.toggleSpeed());
    }

    public Command getAutonomousCommand() {
        // TODO: Add auto commands here
        return null;
    }

    public void onDisabled() {
        Commands.run(this::update)
        .ignoringDisable(true).schedule();
    }

    public void onEnabled() {
        swerve.resetEncoders();
    }

    public void periodic() {
    }    

    public Command update() {
        return Commands.runOnce(() -> FieldConstants.ALLIANCE = DriverStation.getAlliance());
    }

    private void incinerateMotors() {
        Timer.delay(0.25);
        for (CANSparkBase neo : NeoMotorConstants.motors) {
            neo.burnFlash();
            Timer.delay(0.005);
        }
        Timer.delay(0.25);
    }

}
