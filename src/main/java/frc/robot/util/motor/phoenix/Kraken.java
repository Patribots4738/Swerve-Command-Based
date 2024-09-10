package frc.robot.util.motor.phoenix;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.RobotController;

public class Kraken extends TalonFX {

    private double targetPosition = 0.0;
    private double targetVelocity = 0.0;
    private double targetPercent = 0.0;

    private final TalonFXConfiguration talonFXConfigs;
    private AnalogInput turnAbsoluteEncoder = null;

    private final MotionMagicVoltage positionRequest;
    private final MotionMagicVelocityVoltage velocityRequest;

    public Kraken(int id) {
        this(id, false, false, false);
    }

    public Kraken(int id, boolean inverted) {
        this(id, inverted, false, false);
    }

    public Kraken(int id, boolean inverted, boolean useFOC, boolean useAbsoluteEncoder) {
        super(id);
        setInverted(inverted);
        talonFXConfigs = new TalonFXConfiguration();
        positionRequest = new MotionMagicVoltage(0, useFOC, 0, 0, false, false, false);
        velocityRequest = new MotionMagicVelocityVoltage(0, 0, useFOC, 0, 0, false, false, false);
        if (useAbsoluteEncoder) {
            turnAbsoluteEncoder = new AnalogInput(0);
        }
    }

    public void setTargetPosition(double position) {
        setTargetPosition(position, 0, 0);
    }

    public void setTargetPosition(double position, double feedForward) {
        setTargetPosition(position, feedForward, 0);
    }

    public void setTargetPosition(double position, int slot) {
        setTargetPosition(position, 0, slot);
    }

    public void setTargetPosition(double position, double feedForward, int slot) {
        setControl(positionRequest.withPosition(position).withFeedForward(feedForward).withSlot(slot));
        targetPosition = position;
    }

    public void setTargetVelocity(double velocity) {
        setTargetVelocity(velocity, 0, 0);
    }

    public void setTargetVelocity(double velocity, double feedForward) {
        setTargetVelocity(velocity, feedForward, 0);
    }

    public void setTargetVelocity(double velocity, int slot) {
        setTargetVelocity(velocity, 0, slot);
    }

    public void setTargetVelocity(double velocity, double feedForward, int slot) {
        setControl(velocityRequest.withVelocity(velocity).withFeedForward(feedForward).withSlot(slot));
        targetVelocity = velocity;
    }

    public void setPercent(double percent) {
        set(percent);
        targetPercent = percent;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getTargetPercent() {
        return targetPercent;
    }

    // TODO: figure out conversion factors
    public double getPositionAsDouble() {
        return
            (turnAbsoluteEncoder != null)
                ? turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI
                : super.getPosition().refresh().getValue();        
    }
}
