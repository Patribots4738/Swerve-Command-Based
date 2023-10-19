// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.oblarg.oblog.Logger;

/**
 * Do not change the name of this class or the package. This class is one of the main
 * entry points to the robot code and is specified as such in the build.gradle file.
 * 
 * if you change the name of this class or the package, you will need to update the
 * build.gradle file to match. (BUT DON'T DO THAT)
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
        DriverStation.silenceJoystickConnectionWarning(true);

        // Configure logging and config settings
        Logger.configureLoggingAndConfig(robotContainer, false);

    }

    

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Update the logger
        Logger.updateEntries();
    }

     @Override
  public void simulationPeriodic() {
        // Here we calculate the battery voltage based on drawn current.
        // As our robot draws more power from the battery its voltage drops.
        // The estimated voltage is highly dependent on the battery's internal
        // resistance.
        double drawCurrent = robotContainer.getDrivetrain().getDrawnCurrentAmps();
        double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drawCurrent);
        RoboRioSim.setVInVoltage(loadedVoltage);
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        robotContainer.onDisabled();
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        /*
         * This will get the selected command from Smart Dashboard and
         * will run it.
         */
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        /*
         * This makes sure that the autonomous stops running when
         * teleop starts running. 
         * 
         * <p>
         * The onEnabled() method of the {@link RobotContainer} class
         * is called here. This is where you would put code that you
         * want to run when the robot is enabled, such as resetting
         * sensors, etc...
         */
        robotContainer.onEnabled();
        if (autonomousCommand != null)
            autonomousCommand.cancel();

    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}