package frc.robot;

import com.revrobotics.CANSparkMax;

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

public class RobotContainer {

    private final PatriBoxController driver;
    @SuppressWarnings("unused")
    private final PatriBoxController operator;

    private final Swerve swerve;
    @SuppressWarnings("unused")
    private final DriverUI driverUI;
    
    public RobotContainer() {
        driver = new PatriBoxController(OIConstants.DRIVER_CONTROLLER_PORT, OIConstants.DRIVER_DEADBAND);
        operator = new PatriBoxController(OIConstants.OPERATOR_CONTROLLER_PORT, OIConstants.OPERATOR_DEADBAND);

        swerve = new Swerve();
        driverUI = new DriverUI();

        swerve.setDefaultCommand(new Drive(
            swerve,
            driver::getLeftY,
            driver::getLeftX,
            () -> -driver.getRightX(),
            () -> !driver.y().getAsBoolean(),
            () -> !driver.y().getAsBoolean()
        ));

        incinerateMotors();
        configureButtonBindings();

        // Wait wait wait wait for the DS to connect
        // then assign our alliance color
        while (DriverStation.getAlliance() == Alliance.Invalid) {
            DriverStation.refreshData();
        }

        FieldConstants.ALLIANCE = DriverStation.getAlliance();
    }

    private void configureButtonBindings() {

        driver.start().or(driver.back()).onTrue(
            Commands.runOnce(() -> swerve.resetOdometry(
                new Pose2d(
                    swerve.getPose().getTranslation(), 
                    Rotation2d.fromDegrees(
                        FieldConstants.ALLIANCE == Alliance.Red 
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

    public Command getDisabledCommand() {
        return Commands.run(() -> FieldConstants.ALLIANCE = DriverStation.getAlliance()).ignoringDisable(true);
    }

    private void incinerateMotors() {
        Timer.delay(0.25);
        for (CANSparkMax neo : NeoMotorConstants.motors) {
            neo.burnFlash();
            Timer.delay(0.005);
        }
        Timer.delay(0.25);
    }    
}
