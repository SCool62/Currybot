package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {
    private final SingleJointedArmSim physicsSim;

    private final ProfiledPIDController positionPID;
    private final ArmFeedforward feedforward;

    private double appliedVoltage = 0.0;

    public PivotIOSim(double gearRatio, double jKgMetersSquared, double armLengthMeters, double minAngleRads, double maxAngleRads, ProfiledPIDController positionPID, ArmFeedforward feedforward) {
        physicsSim = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(1), gearRatio, jKgMetersSquared, armLengthMeters, minAngleRads, maxAngleRads, true, 0, 0.0);

        this.positionPID = positionPID;
        this.feedforward = feedforward;
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        physicsSim.update(0.02);
        inputs.position = Rotation2d.fromRadians(physicsSim.getAngleRads());
        inputs.angularVelocityRotationsPerSecond = RadiansPerSecond.of(physicsSim.getVelocityRadPerSec()).in(RotationsPerSecond);
        inputs.voltage = appliedVoltage;
        inputs.statorCurrentAmps = physicsSim.getCurrentDrawAmps();
        inputs.supplyCurrentAmps = 0.0;
        inputs.tempC = 0.0;
    }

    @Override
    public void setPositionSetpoint(Rotation2d setpoint) {
        setVoltage(
            positionPID.calculate(physicsSim.getAngleRads(), setpoint.getRadians()) 
            + feedforward.calculate(setpoint.getRadians(), positionPID.getSetpoint().velocity)
        );
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVoltage = voltage;
        physicsSim.setInputVoltage(voltage);
    }

    @Override
    public void resetEncoder(Rotation2d position) {
        physicsSim.setState(position.getRadians(), 0.0);
    }
}
