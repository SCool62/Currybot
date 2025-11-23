package frc.robot.subsystems.colorsensor;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.util.Color;

public interface ColorSensorIO {
    @AutoLog
    public static class ColorSensorIOInputs {
        public String color = "";
    }

    void updateInputs(ColorSensorIOInputs inputs);
}
