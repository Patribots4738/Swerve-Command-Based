package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CmdPatriBoxController extends CommandXboxController{
    private double deadband = 0.1;
    
    public CmdPatriBoxController(int port, double deadband) {
        super(port);
        this.deadband = deadband;
    }

    @Override
    public double getLeftX() {
        return MathUtil.applyDeadband(super.getLeftX(), deadband);
    }

    @Override
    public double getLeftY() {
        return MathUtil.applyDeadband(super.getLeftY(), deadband);
    }

    @Override
    public double getRightX() {
        return MathUtil.applyDeadband(super.getRightX(), deadband);
    }

    @Override
    public double getRightY() {
        return MathUtil.applyDeadband(super.getRightY(), deadband);
    }    
}
