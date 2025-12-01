package frc.robot.subsystems.colorsensor;

import org.littletonrobotics.junction.AutoLog;

public interface ColorSensorIO {
  @AutoLog
  public static class ColorSensorIOInputs {
    /**
     * The color that the sensor sees, as a hex code
     */
    public String color = "";
  }

  /**
   * Updates the passed-in inputs with the sensor's data
   * @param inputs the inputs to be updated
   */
  void updateInputs(ColorSensorIOInputs inputs);
}
