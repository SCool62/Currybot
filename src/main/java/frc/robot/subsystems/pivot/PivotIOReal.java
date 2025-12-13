package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class PivotIOReal implements PivotIO {
  private final TalonFX motor;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temperature;

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);

  private Rotation2d setpoint = Rotation2d.kZero;

  /**
   * Creates a new pivot with a TalonFX-controlled motor
   *
   * @param motorId the ID of the motor on the CAN bus
   * @param config the motor's configuration
   */
  public PivotIOReal(int motorId, TalonFXConfiguration config) {
    motor = new TalonFX(motorId);
    motor.getConfigurator().apply(config);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    voltage = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    temperature = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, voltage, statorCurrent, supplyCurrent, temperature);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        position, velocity, voltage, statorCurrent, supplyCurrent, temperature);

    inputs.position = new Rotation2d(position.getValue());
    inputs.angularVelocityRotationsPerSecond = velocity.getValue().in(RotationsPerSecond);
    inputs.voltage = voltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempC = temperature.getValueAsDouble();
  }

  @Override
  public void setPositionSetpoint(Rotation2d setpoint) {
    this.setpoint = setpoint;
    motor.setControl(positionVoltage.withPosition(setpoint.getMeasure()));
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void resetEncoder(Rotation2d position) {
    motor.setPosition(position.getMeasure());
  }

  @Override
  public Rotation2d getSetpoint() {
      return setpoint;
  }
}
