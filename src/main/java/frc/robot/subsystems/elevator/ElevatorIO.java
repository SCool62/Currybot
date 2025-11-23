package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public class ElevatorIOInputs {
    public double leaderPositionMeters = 0.0;
    public double leaderVelocityMetersPerSec = 0.0;
    public double leaderStatorCurrentAmps = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;
    public double leaderVoltage = 0.0;
    public double leaderTempC = 0.0;

    public double followerPositionMeters = 0.0;
    public double followerVelocityMetersPerSec = 0.0;
    public double followerStatorCurrentAmps = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerVoltage = 0.0;
    public double followerTempC = 0.0;
  }

  void updateInputs(ElevatorIOInputs inputs);

  void setPositionSetpoint(double positionMeters);

  void setVoltage(double voltage);

  void setEncoderPosition(double positionMeters);
}
