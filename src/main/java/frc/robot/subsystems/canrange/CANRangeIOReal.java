package frc.robot.subsystems.canrange;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;

public class CANRangeIOReal implements CANRangeIO {
  private final CANrange canrange;

  private final StatusSignal<Boolean> isDetected;
  private final StatusSignal<Distance> distance;

  /**
   * Creates a new CANrange
   *
   * @param deviceId the ID of the device on the CAN bus
   * @param config the sensor's configuration
   */
  public CANRangeIOReal(int deviceId, CANrangeConfiguration config) {
    this.canrange = new CANrange(deviceId);

    isDetected = canrange.getIsDetected();
    distance = canrange.getDistance();

    canrange.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, isDetected, distance);
    canrange.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(CANRangeIOInputs inputs) {
    BaseStatusSignal.refreshAll(isDetected, distance);

    inputs.distanceMeters = distance.getValue().in(Meters);
    inputs.isDetected = isDetected.getValue();
  }
}
