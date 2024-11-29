package frc.robot.subsystems.drive.gyro;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.hardware.phoenix.Pigeon2Custom;

public class Gyro implements GyroIO {
    private final Pigeon2Custom pigeon;
    private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    public Gyro() {
        pigeon = new Pigeon2Custom(DriveConstants.GYRO_CAN_ID);
    }

    public Rotation2d getYawRotation2D() {
        return inputs.yawRotation2d;
    }

    public double getYaw() {
        return inputs.yawRotation2d.getRadians();
    }

    public double getPitch() {
        return inputs.pitchRads;
    }

    public double getRoll() {
        return inputs.rollRads;
    }

    public double getYawVelocity() {
        return inputs.yawVelocityRadsPerSec;
    }

    public double getPitchVelocity() {
        return inputs.yawVelocityRadsPerSec;
    }

    public double getRollVelocity() {
        return inputs.yawVelocityRadsPerSec;
    }

    public void setYaw(double degrees) {
        pigeon.setYaw(degrees);
    }

    public void processInputs() {
        Logger.processInputs("SubsystemInputs/Gyro", inputs);
    }

    public void updateInputs() {
        // refresh all status signals
        inputs.isConnected = pigeon.isConnected();
        inputs.yawRotation2d = pigeon.getRotation2d();
        inputs.pitchRads = pigeon.getPitchRadians();
        inputs.rollRads = pigeon.getRollRadians();
        inputs.yawVelocityRadsPerSec = pigeon.getYawVelocityRadiansPerSec();
        inputs.pitchVelocityRadsPerSec = pigeon.getPitchVelocityRadiansPerSec();
        inputs.rollVelocityRadsPerSec = pigeon.getRollVelocityRadiansPerSec();
    }
}
