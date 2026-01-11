package frc.robot.subsystems.drive.module;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ModuleIOInputsAutoLogged extends ModuleIO.ModuleIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("DriverMotorConnected", driverMotorConnected);
    table.put("DrivePositionFlipped", drivePositionFlipped);
    table.put("DrivePositionMeters", drivePositionMeters);
    table.put("DriveVelocityMPS", driveVelocityMPS);
    table.put("DriveAppliedVolts", driveAppliedVolts);
    table.put("DriveSupplyCurrentAmps", driveSupplyCurrentAmps);
    table.put("DriveStatorCurrentAmps", driveStatorCurrentAmps);
    table.put("DriveTempCelcius", driveTempCelcius);
    table.put("TurnMotorConnected", turnMotorConnected);
    table.put("TurnInternalPositionRads", turnInternalPositionRads);
    table.put("TurnInternalVelocityRadsPerSec", turnInternalVelocityRadsPerSec);
    table.put("TurnAppliedVolts", turnAppliedVolts);
    table.put("TurnSupplyCurrentAmps", turnSupplyCurrentAmps);
    table.put("TurnStatorCurrentAmps", turnStatorCurrentAmps);
    table.put("TurnTempCelcius", turnTempCelcius);
    table.put("TurnEncoderConnected", turnEncoderConnected);
    table.put("TurnEncoderPositionRads", turnEncoderPositionRads);
    table.put("TurnEncoderAbsPositionRads", turnEncoderAbsPositionRads);
    table.put("TurnEncoderVelocityRadsPerSec", turnEncoderVelocityRadsPerSec);
  }

  @Override
  public void fromLog(LogTable table) {
    driverMotorConnected = table.get("DriverMotorConnected", driverMotorConnected);
    drivePositionFlipped = table.get("DrivePositionFlipped", drivePositionFlipped);
    drivePositionMeters = table.get("DrivePositionMeters", drivePositionMeters);
    driveVelocityMPS = table.get("DriveVelocityMPS", driveVelocityMPS);
    driveAppliedVolts = table.get("DriveAppliedVolts", driveAppliedVolts);
    driveSupplyCurrentAmps = table.get("DriveSupplyCurrentAmps", driveSupplyCurrentAmps);
    driveStatorCurrentAmps = table.get("DriveStatorCurrentAmps", driveStatorCurrentAmps);
    driveTempCelcius = table.get("DriveTempCelcius", driveTempCelcius);
    turnMotorConnected = table.get("TurnMotorConnected", turnMotorConnected);
    turnInternalPositionRads = table.get("TurnInternalPositionRads", turnInternalPositionRads);
    turnInternalVelocityRadsPerSec = table.get("TurnInternalVelocityRadsPerSec", turnInternalVelocityRadsPerSec);
    turnAppliedVolts = table.get("TurnAppliedVolts", turnAppliedVolts);
    turnSupplyCurrentAmps = table.get("TurnSupplyCurrentAmps", turnSupplyCurrentAmps);
    turnStatorCurrentAmps = table.get("TurnStatorCurrentAmps", turnStatorCurrentAmps);
    turnTempCelcius = table.get("TurnTempCelcius", turnTempCelcius);
    turnEncoderConnected = table.get("TurnEncoderConnected", turnEncoderConnected);
    turnEncoderPositionRads = table.get("TurnEncoderPositionRads", turnEncoderPositionRads);
    turnEncoderAbsPositionRads = table.get("TurnEncoderAbsPositionRads", turnEncoderAbsPositionRads);
    turnEncoderVelocityRadsPerSec = table.get("TurnEncoderVelocityRadsPerSec", turnEncoderVelocityRadsPerSec);
  }

  public ModuleIOInputsAutoLogged clone() {
    ModuleIOInputsAutoLogged copy = new ModuleIOInputsAutoLogged();
    copy.driverMotorConnected = this.driverMotorConnected;
    copy.drivePositionFlipped = this.drivePositionFlipped;
    copy.drivePositionMeters = this.drivePositionMeters;
    copy.driveVelocityMPS = this.driveVelocityMPS;
    copy.driveAppliedVolts = this.driveAppliedVolts;
    copy.driveSupplyCurrentAmps = this.driveSupplyCurrentAmps;
    copy.driveStatorCurrentAmps = this.driveStatorCurrentAmps;
    copy.driveTempCelcius = this.driveTempCelcius;
    copy.turnMotorConnected = this.turnMotorConnected;
    copy.turnInternalPositionRads = this.turnInternalPositionRads;
    copy.turnInternalVelocityRadsPerSec = this.turnInternalVelocityRadsPerSec;
    copy.turnAppliedVolts = this.turnAppliedVolts;
    copy.turnSupplyCurrentAmps = this.turnSupplyCurrentAmps;
    copy.turnStatorCurrentAmps = this.turnStatorCurrentAmps;
    copy.turnTempCelcius = this.turnTempCelcius;
    copy.turnEncoderConnected = this.turnEncoderConnected;
    copy.turnEncoderPositionRads = this.turnEncoderPositionRads;
    copy.turnEncoderAbsPositionRads = this.turnEncoderAbsPositionRads;
    copy.turnEncoderVelocityRadsPerSec = this.turnEncoderVelocityRadsPerSec;
    return copy;
  }
}
