package frc.robot.subsystems.colorsensor;

import org.littletonrobotics.junction.AutoLog;

public interface ColorSensorIO {
  @AutoLog
  public static class ColorSensorIOInputs {
    public String color = "";
  }

  void updateInputs(ColorSensorIOInputs inputs);
}
