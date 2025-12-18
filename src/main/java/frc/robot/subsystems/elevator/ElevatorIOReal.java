package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOReal implements ElevatorIO {
  private TalonFX leader = new TalonFX(0, "*");
  private TalonFX follower = new TalonFX(0, "*");

  // Conversion from angle to distance happens in sensor to mechanism ratio
  private final BaseStatusSignal leaderPositionMeters = leader.getPosition();
  private final BaseStatusSignal leaderVelocityMetersPerSec = leader.getVelocity();
  private final StatusSignal<Voltage> leaderVoltage = leader.getMotorVoltage();
  private final StatusSignal<Current> leaderStatorCurrent = leader.getStatorCurrent();
  private final StatusSignal<Current> leaderSupplyCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Temperature> leaderTemp = leader.getDeviceTemp();

  private final BaseStatusSignal followerPositionMeters = follower.getPosition();
  private final BaseStatusSignal followerVelocityMetersPerSec = follower.getVelocity();
  private final StatusSignal<Voltage> followerVoltage = follower.getMotorVoltage();
  private final StatusSignal<Current> followerStatorCurrent = follower.getStatorCurrent();
  private final StatusSignal<Current> followerSupplyCurrent = follower.getSupplyCurrent();
  private final StatusSignal<Temperature> followerTemp = follower.getDeviceTemp();

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withEnableFOC(true);

  public ElevatorIOReal() {
    // In here is where we would do configs but i'm lazy

    // On a real robot these would all have different ids and you would put it here
    follower.setControl(new Follower(0, false));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPositionMeters,
        leaderVelocityMetersPerSec,
        leaderVoltage,
        leaderStatorCurrent,
        leaderSupplyCurrent,
        leaderTemp,
        followerPositionMeters,
        followerVelocityMetersPerSec,
        followerVoltage,
        followerStatorCurrent,
        followerSupplyCurrent,
        followerTemp);

    inputs.leaderPositionMeters = leaderPositionMeters.getValueAsDouble();
    inputs.leaderVelocityMetersPerSec = leaderVelocityMetersPerSec.getValueAsDouble();
    inputs.leaderVoltage = leaderVoltage.getValueAsDouble();
    inputs.leaderStatorCurrentAmps = leaderStatorCurrent.getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leaderSupplyCurrent.getValueAsDouble();
    inputs.leaderTempC = leaderTemp.getValueAsDouble();

    inputs.followerPositionMeters = followerPositionMeters.getValueAsDouble();
    inputs.followerVelocityMetersPerSec = followerVelocityMetersPerSec.getValueAsDouble();
    inputs.followerVoltage = followerVoltage.getValueAsDouble();
    inputs.followerStatorCurrentAmps = followerStatorCurrent.getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerSupplyCurrent.getValueAsDouble();
    inputs.followerTempC = followerTemp.getValueAsDouble();
  }

  @Override
  public void setPositionSetpoint(double positionMeters) {
    leader.setControl(motionMagicVoltage.withPosition(positionMeters));
  }

  @Override
  public void setVoltage(double voltage) {
    leader.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void setEncoderPosition(double positionMeters) {
    leader.setPosition(positionMeters);
  }
}
