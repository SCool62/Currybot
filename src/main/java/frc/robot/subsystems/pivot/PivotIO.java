package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public Rotation2d position = new Rotation2d();
    public double angularVelocityRotationsPerSecond = 0.0;
    public double voltage = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempC = 0.0;
  }

  /**
   * Updates the passed-in inputs with the motor's data
   *
   * @param inputs the inputs to be updated
   */
  void updateInputs(PivotIOInputs inputs);

  /**
   * Runs the pivot to the specified setpoint
   *
   * @param setpoint
   */
  void setPositionSetpoint(Rotation2d setpoint);

  /**
   * Sets the voltage that the motor recieves
   *
   * @param voltage
   */
  void setVoltage(double voltage);

  /**
   * Resets the motor's encoder to the specified position
   *
   * @param position
   */
  void resetEncoder(Rotation2d position);

  /** Resets the motor's encoder to zero */
  default void resetEncoder() {
    resetEncoder(Rotation2d.kZero);
  }
}
