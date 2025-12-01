package frc.robot.subsystems.canrange;

import org.littletonrobotics.junction.AutoLog;

public interface CANRangeIO {
  @AutoLog
  public static class CANRangeIOInputs {
    /** Like a beambreak detection value Threshold set in CANrange configuration */
    public boolean isDetected = false;

    /** The distance from the sensor to the detected object */
    public double distanceMeters = 0.0;
  }

  /**
   * Updates the passed-in inputs with the sensor's data
   *
   * @param inputs the inputs to be updated
   */
  void updateInputs(CANRangeIOInputs inputs);
}
