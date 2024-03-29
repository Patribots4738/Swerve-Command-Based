package frc.robot;

import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import io.github.oblarg.oblog.Logger;
import monologue.Monologue;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private Command autonomousCommand;

    private RobotContainer robotContainer;

    @Override
    public void robotInit() { 
        robotContainer = new RobotContainer();

        Monologue.setupMonologue(robotContainer, "Robot", false, false);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Used for items like diagnostics
     * ran during disabled, autonomous, teleoperated and test. :D
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        //Logger.updateEntries();
        Monologue.updateAll();
        CommandScheduler.getInstance().run();
        robotContainer.periodic();
        DriverUI.previousTimestmap = DriverUI.currentTimestamp;
        DriverUI.currentTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void disabledInit() {
        robotContainer.onDisabled();
    }

    @Override
    public void disabledPeriodic() { }

    @Override
    public void autonomousInit() { 
        robotContainer.onEnabled();
        autonomousCommand = robotContainer.getAutonomousCommand();
        
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() { }
    
    @Override
    public void teleopInit() {
        robotContainer.onEnabled();
        // Stop our autonomous command if it is still running.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() { }
    
    @Override
    public void testInit() { 
        robotContainer.onEnabled();
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() { }

    @Override
    public void simulationInit() { }

    @Override
    public void simulationPeriodic() { 
        REVPhysicsSim.getInstance().run();
    }
}
