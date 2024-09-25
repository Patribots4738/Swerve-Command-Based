package frc.robot.subsystems.test;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.motor.phoenix.Kraken;

public class KrakenTest extends SubsystemBase implements KrakenTestIO {
    
    private Kraken motor = new Kraken(20, false, true);
    KrakenTestIOInputsAutoLogged inputs = new KrakenTestIOInputsAutoLogged();

    public KrakenTest() {
        motor.setGains(0.11, 0, 0, 0.1, 0.12);
        motor.setVelocityConversionFactor(60.0);
    }

    @Override
    public void periodic() {
        updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/KrakenTest", inputs);
        Logger.recordOutput("Subsystems/KrakenTest/Position", inputs.positionRotations);
        Logger.recordOutput("Subsystems/KrakenTest/Velocity", inputs.velocityRPM);
        Logger.recordOutput("Subsystems/KrakenTest/TargetVelocity", inputs.targetVelocityRPM);
        Logger.recordOutput("Subsystems/KrakenTest/TargetPercent", inputs.targetPercent);
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

    public void updateInputs(KrakenTestIOInputs inputs) {
        inputs.positionRotations = motor.getPositionAsDouble();
        inputs.targetPositionRotations = motor.getTargetPosition();
        inputs.velocityRPM = motor.getVelocityAsDouble();
        inputs.targetVelocityRPM = motor.getTargetVelocity();
        inputs.appliedVolts = motor.getVoltageAsDouble();
        inputs.targetPercent = motor.getTargetPercent();
        inputs.statorCurrentAmps = motor.getStatorCurrentAsDouble();
    }

}
