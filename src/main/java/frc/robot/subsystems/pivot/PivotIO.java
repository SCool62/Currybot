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

  void updateInputs(PivotIOInputs inputs);

  void setPositionSetpoint(Rotation2d setpoint);

  void setVoltage(double voltage);

  void resetEncoder(Rotation2d position);

  default void resetEncoder() {
    resetEncoder(Rotation2d.kZero);
  }
}
