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

  void updateInputs(RollerIOInputs inputs);

  void setPositionSetpoint(Rotation2d setpoint);

  void setVelocitySetpoint(double velocityRotationsPerSecond);

  void setVoltage(double voltage);

  void resetEncoder(Rotation2d position);
}
