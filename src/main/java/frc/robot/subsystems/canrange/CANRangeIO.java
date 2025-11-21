package frc.robot.subsystems.canrange;

import org.littletonrobotics.junction.AutoLog;

public interface CANRangeIO {
    @AutoLog
    public static class CANRangeIOInputs {
        public boolean isDetected = false;
        public double distanceMeters = 0.0;
    }

    void updateInputs(CANRangeIOInputs inputs);
}
