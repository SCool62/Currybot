package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  // Technically x44 but DCmotor doesn't have it
  // TODO: ACTUAL VALUES
  private ElevatorSim physicsSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(2), 
          ElevatorSubsystem.GEAR_RATIO, 
          Units.lbsToKilograms(12.399), 
          // What should drum radius be?????? Its a beltivator
          0.0, 
          0.0, 
          Units.inchesToMeters(35.126), 
          true, 
          0.0);

  // TODO: TUNE IN SIM
  private ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);
  private ProfiledPIDController pid =
      new ProfiledPIDController(0, 0, 0, new Constraints(5.0, 10.0));

  private double currentAppliedVoltage = 0.0;

  public ElevatorIOSim() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    physicsSim.update(0.02);
    inputs.leaderPositionMeters = physicsSim.getPositionMeters();
    inputs.leaderVoltage = currentAppliedVoltage;
    inputs.leaderStatorCurrentAmps = physicsSim.getCurrentDrawAmps();
    inputs.leaderVelocityMetersPerSec = physicsSim.getVelocityMetersPerSecond();
    inputs.leaderSupplyCurrentAmps = Double.NEGATIVE_INFINITY;
    inputs.leaderTempC = Double.NEGATIVE_INFINITY;

    inputs.followerPositionMeters = inputs.leaderPositionMeters;
    inputs.followerVoltage = inputs.leaderVoltage;
    inputs.followerStatorCurrentAmps = inputs.leaderStatorCurrentAmps;
    inputs.followerVelocityMetersPerSec = inputs.leaderVelocityMetersPerSec;
    inputs.followerSupplyCurrentAmps = Double.NEGATIVE_INFINITY;
    inputs.followerTempC = Double.NEGATIVE_INFINITY;
  }

  @Override
  public void setVoltage(double voltage) {
    physicsSim.setInputVoltage(voltage);
  }

  @Override
  public void setPositionSetpoint(double positionMeters) {
    setVoltage(
        pid.calculate(physicsSim.getPositionMeters(), positionMeters)
            + feedforward.calculate(pid.getSetpoint().velocity));
  }

  @Override
  public void setEncoderPosition(double positionMeters) {
    physicsSim.setState(positionMeters, 0.0);
  }
}
