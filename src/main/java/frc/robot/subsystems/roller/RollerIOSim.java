package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerIOSim implements RollerIO {
  private final DCMotorSim physicsSim;

  private final SimpleMotorFeedforward feedforward;
  private final ProfiledPIDController positionPID;
  private final PIDController velocityPID;

  /**
   * Creates a simulated roller with 1 krakenX60 with FOC
   *
   * @param jKgMetersSquared the moment of inertia of the mechanism
   * @param gearing the gear ratio from rotor to mechanism
   * @param feedforward the roller's feedforward controller
   * @param positionPID the PID controlling the roller's position
   * @param velocityPID the PID controlling the roller's velocity
   */
  public RollerIOSim(
      double jKgMetersSquared,
      double gearing,
      SimpleMotorFeedforward feedforward,
      ProfiledPIDController positionPID,
      PIDController velocityPID) {
    physicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), jKgMetersSquared, gearing),
            DCMotor.getKrakenX60Foc(1));

    this.feedforward = feedforward;
    this.positionPID = positionPID;
    this.velocityPID = velocityPID;
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    physicsSim.update(0.02);
    inputs.position = new Rotation2d(physicsSim.getAngularPosition());
    inputs.velocityRotationsPerSecond = physicsSim.getAngularVelocity().in(RotationsPerSecond);
    inputs.voltage = physicsSim.getInputVoltage();
    inputs.statorCurrentAmps = physicsSim.getCurrentDrawAmps();
    inputs.supplyCurrentAmps = Double.NEGATIVE_INFINITY;
    inputs.tempC = Double.NEGATIVE_INFINITY;
  }

  @Override
  public void setPositionSetpoint(Rotation2d setpoint) {
    physicsSim.setInputVoltage(
        positionPID.calculate(physicsSim.getAngularPositionRotations(), setpoint.getRotations())
            + feedforward.calculate(positionPID.getSetpoint().velocity));
  }

  @Override
  public void setVelocitySetpoint(double velocityRotationsPerSecond) {
    physicsSim.setInputVoltage(
        velocityPID.calculate(
                physicsSim.getAngularVelocity().in(RotationsPerSecond), velocityRotationsPerSecond)
            + feedforward.calculate(velocityRotationsPerSecond));
  }

  @Override
  public void setVoltage(double voltage) {
    physicsSim.setInputVoltage(voltage);
  }

  @Override
  public void resetEncoder(Rotation2d position) {
    physicsSim.setAngle(position.getRadians());
  }
}
