package frc.robot.subsystems.canrange;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.sim.CANrangeSimState;
import edu.wpi.first.units.measure.Distance;

public class CANRangeIOSim implements CANRangeIO {
  private final CANrange canrange;
  // Must admit not entirely sure how to use this but i'm going to try to figure it out.
  private CANrangeSimState simState;

  private final StatusSignal<Distance> distance;
  private final StatusSignal<Boolean> isDetected;

  public CANRangeIOSim(int deviceId, CANrangeConfiguration config) {
    canrange = new CANrange(deviceId);

    distance = canrange.getDistance();
    isDetected = canrange.getIsDetected();

    simState = canrange.getSimState();
  }

  @Override
  public void updateInputs(CANRangeIOInputs inputs) {
    inputs.distanceMeters = distance.getValue().in(Meters);
    inputs.isDetected = isDetected.getValue();
  }

  public void setDistance(double distanceMeters) {
    simState.setDistance(distanceMeters);
  }
}
