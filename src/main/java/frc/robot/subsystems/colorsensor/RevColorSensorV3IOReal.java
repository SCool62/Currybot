package frc.robot.subsystems.colorsensor;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;

public class RevColorSensorV3IOReal implements ColorSensorIO {

  private final ColorSensorV3 sensor;

  public RevColorSensorV3IOReal(I2C.Port port) {
    sensor = new ColorSensorV3(port);
  }

  @Override
  public void updateInputs(ColorSensorIOInputs inputs) {
    inputs.color = sensor.getColor().toHexString();
  }
}
