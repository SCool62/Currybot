package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class DoubleMotorRollerIOReal implements RollerIO {
    private final TalonFX leader;
    private final TalonFX follower;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Voltage> voltage;

  private VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);
  private PositionVoltage positionVoltage =
      new PositionVoltage(0.0).withEnableFOC(true).withSlot(1);
  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  /**
   * Creates a roller controlled by 2 TalonFX controlled motors
   *
   * @param leaderID the ID of the lead motor on the CAN bus
   * @param followerID the ID of the follower motor on the CAN bus
   * @param opposeMasterDirection if the follower should move opposite the leader
   * @param config the motor's configuration
   * @implNote If using position control, put position PID and feedforward gains on slot 1
   */
  public DoubleMotorRollerIOReal(int leaderID, int followerID, boolean opposeMasterDirection, TalonFXConfiguration config) {
    leader = new TalonFX(leaderID);
    follower = new TalonFX(followerID);

    position = leader.getPosition();
    velocity = leader.getVelocity();
    statorCurrent = leader.getStatorCurrent();
    supplyCurrent = leader.getSupplyCurrent();
    temperature = leader.getDeviceTemp();
    voltage = leader.getMotorVoltage();

    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, statorCurrent, supplyCurrent, temperature, voltage);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();

    follower.setControl(new Follower(leaderID, opposeMasterDirection));
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        position, velocity, statorCurrent, supplyCurrent, temperature, voltage);

    inputs.position = new Rotation2d(position.getValue());
    inputs.velocityRotationsPerSecond = velocity.getValue().in(RotationsPerSecond);
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempC = temperature.getValueAsDouble();
    inputs.voltage = voltage.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    leader.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void setPositionSetpoint(Rotation2d setpoint) {
    leader.setControl(positionVoltage.withPosition(setpoint.getMeasure()));
  }

  @Override
  public void setVelocitySetpoint(double velocityRotationsPerSecond) {
    leader.setControl(velocityVoltage.withVelocity(velocityRotationsPerSecond));
  }

  @Override
  public void resetEncoder(Rotation2d position) {
    leader.setPosition(position.getMeasure());
  }
}
