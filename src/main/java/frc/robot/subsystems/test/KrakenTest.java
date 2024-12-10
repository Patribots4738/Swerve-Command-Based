package frc.robot.subsystems.test;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.hardware.phoenix.Kraken;

public class KrakenTest extends SubsystemBase implements KrakenTestIO {
    
    private Kraken motor;
    KrakenTestIOInputsAutoLogged inputs = new KrakenTestIOInputsAutoLogged();

    public KrakenTest() {
        motor = new Kraken(20, "rio", true, false);
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

    public Command setPosition(DoubleSupplier position) {
        return runOnce(() -> motor.setTargetPosition(position.getAsDouble()));
    }

    public Command setVelocity(DoubleSupplier velocity) {
        return runOnce(() -> motor.setTargetVelocity(velocity.getAsDouble()));
    }

    public Command setPercent(DoubleSupplier percent) {
        return runOnce(() -> motor.setPercentOutput(percent.getAsDouble()));
    }

    public Command setVoltage(DoubleSupplier volts) {
        return runOnce(() -> motor.setVoltageOutput(volts.getAsDouble()));
    }

    public Command setCurrent(DoubleSupplier amps) {
        return runOnce(() -> motor.setTorqueCurrentOutput(amps.getAsDouble()));
    }

    public double getPosition() {
        return inputs.positionRotations;
    }

    public void updateInputs(KrakenTestIOInputs inputs) {
        inputs.krakenConnected = motor.refreshSignals().isOK();
        inputs.positionRotations = motor.getPositionAsDouble();
        inputs.targetPositionRotations = motor.getTargetPosition();
        inputs.velocityRPM = motor.getVelocityAsDouble();
        inputs.targetVelocityRPM = motor.getTargetVelocity();
        inputs.appliedVolts = motor.getVoltageAsDouble();
        inputs.targetPercent = motor.getTargetPercent();
        inputs.supplyCurrentAmps = motor.getSupplyCurrentAsDouble();
        inputs.tempCelcius = motor.getTemperatureAsDouble();
    }

}
