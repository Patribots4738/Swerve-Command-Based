package frc.robot.subsystems.drive.gyro;

import frc.robot.util.hardware.phoenix.Pigeon2Custom;

public class GyroPigeon2IO implements GyroIO {
    private final Pigeon2Custom pigeon;

    public GyroPigeon2IO(int canId) {
        pigeon = new Pigeon2Custom(canId);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        // refresh all status signals
        inputs.isConnected = pigeon.refreshSignals().isOK();
        inputs.yawRotation2d = pigeon.getRotation2d();
        inputs.pitchRads = pigeon.getPitchRadians();
        inputs.rollRads = pigeon.getRollRadians();
        inputs.yawVelocityRadsPerSec = pigeon.getYawVelocityRadiansPerSec();
        inputs.pitchVelocityRadsPerSec = pigeon.getPitchVelocityRadiansPerSec();
        inputs.rollVelocityRadsPerSec = pigeon.getRollVelocityRadiansPerSec();
    }

    @Override
    public void setYaw(double degrees) {
        pigeon.setYaw(degrees);
    }

}
