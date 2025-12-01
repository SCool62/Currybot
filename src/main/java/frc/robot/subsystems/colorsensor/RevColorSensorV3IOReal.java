package frc.robot.subsystems.colorsensor;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;

public class RevColorSensorV3IOReal implements ColorSensorIO {

  private final ColorSensorV3 sensor;

  /**
   * Creates a new Rev Color Sensor V3
   * @param port the I2C port the sensor is plugged into
   */
  public RevColorSensorV3IOReal(I2C.Port port) {
    sensor = new ColorSensorV3(port);
  }

  @Override
  public void updateInputs(ColorSensorIOInputs inputs) {
    inputs.color = sensor.getColor().toHexString();
  }
}
