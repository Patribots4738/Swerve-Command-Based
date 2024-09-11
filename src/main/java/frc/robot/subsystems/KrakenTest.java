package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.motor.phoenix.Kraken;

public class KrakenTest extends SubsystemBase {
    
    private Kraken motor = new Kraken(10, false, true);

    public KrakenTest() {
        motor.setStatorCurrentLimit(20.0);
        motor.setPID(0.5, 0, 0.2);
    }

    public Command setPosition(double position) {
        return Commands.runOnce(() -> motor.setTargetPosition(position));
    }

    public Command setVelocity(double velocity) {
        return Commands.runOnce(() -> motor.setTargetVelocity(velocity));
    }

    public Command setPercent(double percent) {
        return Commands.runOnce(() -> motor.set(percent));
    }

}
