package frc.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public class RollerIOInputs {
    public double velocityRotationsPerSecond = 0.0;
    public Rotation2d position = new Rotation2d();
    public double voltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double tempC = 0.0;
  }

  /**
   * Updates the passed-in inputs with the motor's data
   * @param inputs the inputs to be updated
   */
  void updateInputs(RollerIOInputs inputs);

  /**
   * Runs the motor to the specified position
   * @param setpoint
   */
  void setPositionSetpoint(Rotation2d setpoint);

  /**
   * Runs the motor at the specified velocity
   * @param velocityRotationsPerSecond
   */
  void setVelocitySetpoint(double velocityRotationsPerSecond);

  /**
   * Sets the voltage that the motor recieves
   * @param voltage
   */
  void setVoltage(double voltage);

  /**
   * Resets the motor's encoder to the specified position
   * @param position
   */
  void resetEncoder(Rotation2d position);

  /**
   * Resets the motor's encoder to zero
   */
  default void resetEncoder() {
    resetEncoder(Rotation2d.kZero);
  }
}
