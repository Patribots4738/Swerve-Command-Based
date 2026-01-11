package frc.robot.subsystems.drive.gyro;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("IsConnected", isConnected);
    table.put("YawRotation2d", yawRotation2d);
    table.put("PitchRads", pitchRads);
    table.put("RollRads", rollRads);
    table.put("YawVelocityRadsPerSec", yawVelocityRadsPerSec);
    table.put("PitchVelocityRadsPerSec", pitchVelocityRadsPerSec);
    table.put("RollVelocityRadsPerSec", rollVelocityRadsPerSec);
  }

  @Override
  public void fromLog(LogTable table) {
    isConnected = table.get("IsConnected", isConnected);
    yawRotation2d = table.get("YawRotation2d", yawRotation2d);
    pitchRads = table.get("PitchRads", pitchRads);
    rollRads = table.get("RollRads", rollRads);
    yawVelocityRadsPerSec = table.get("YawVelocityRadsPerSec", yawVelocityRadsPerSec);
    pitchVelocityRadsPerSec = table.get("PitchVelocityRadsPerSec", pitchVelocityRadsPerSec);
    rollVelocityRadsPerSec = table.get("RollVelocityRadsPerSec", rollVelocityRadsPerSec);
  }

  public GyroIOInputsAutoLogged clone() {
    GyroIOInputsAutoLogged copy = new GyroIOInputsAutoLogged();
    copy.isConnected = this.isConnected;
    copy.yawRotation2d = this.yawRotation2d;
    copy.pitchRads = this.pitchRads;
    copy.rollRads = this.rollRads;
    copy.yawVelocityRadsPerSec = this.yawVelocityRadsPerSec;
    copy.pitchVelocityRadsPerSec = this.pitchVelocityRadsPerSec;
    copy.rollVelocityRadsPerSec = this.rollVelocityRadsPerSec;
    return copy;
  }
}
