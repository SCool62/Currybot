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

  /**
   * Updates the passed-in inputs with the motors' data
   * @param inputs the inputs to be updated
   */
  void updateInputs(ElevatorIOInputs inputs);

  /**
   * Runs the elevator to the specified setpoint
   * @param positionMeters the setpoint
   */
  void setPositionSetpoint(double positionMeters);

  /**
   * Sets the voltage that the motors recieve
   * @param voltage
   */
  void setVoltage(double voltage);

  /**
   * Resets the motors' encoders to the specified position 
   * @param positionMeters
   */
  void setEncoderPosition(double positionMeters);

  /**
   * Resets the motors' encoders to zero
   */
  default void setEncoderPosition() {
    setEncoderPosition(0.0);
  }
}
