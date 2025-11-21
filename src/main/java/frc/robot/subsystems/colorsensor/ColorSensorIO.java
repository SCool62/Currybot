package frc.robot.subsystems.colorsensor;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.util.Color;

public interface ColorSensorIO {
    @AutoLog
    public static class ColorSensorIOInputs {
        public Color color = Color.kWhite;
    }

    void updateInputs(ColorSensorIOInputs inputs);
}
