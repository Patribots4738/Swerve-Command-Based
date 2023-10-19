package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class CmdPatriBoxController extends CommandXboxController{
    private double deadband = 0.1;
    
    public CmdPatriBoxController(int port, double deadband) {
        super(port);
        this.deadband = deadband;
    }

    @Override
    public double getLeftX() {
        return MathUtil.applyDeadband(
            super.getLeftX(), 
            deadband) * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    }

    @Override
    public double getLeftY() {
        return MathUtil.applyDeadband(
            super.getLeftY(), 
            deadband) * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    }

    @Override
    public double getRightX() {
        return MathUtil.applyDeadband(
            super.getRightX(), 
            deadband) * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    }

    @Override
    public double getRightY() {
        return MathUtil.applyDeadband(
            super.getRightY(), 
            deadband) * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    }    
}
