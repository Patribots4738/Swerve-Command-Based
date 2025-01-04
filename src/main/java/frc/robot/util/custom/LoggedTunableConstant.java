package frc.robot.util.custom;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LoggedTunableConstant extends LoggedDashboardNumber {

    private double previousValue;
    
    public LoggedTunableConstant(String key, double defaultValue) {
        super("Constants/" + key, defaultValue);
        this.previousValue = defaultValue;
    }

    public LoggedTunableConstant(String key) {
        this(key, 0.0);
    }

    public boolean ifChanged() {
        return get() != previousValue;
    }

    public Trigger onChanged() {
        return new Trigger(this::ifChanged);
    }

    public Trigger onChanged(Command command) {
        return this.onChanged().onTrue(command);
    }
    
    @Override
    public void periodic() {
        previousValue = get();
        super.periodic();
    }
}
