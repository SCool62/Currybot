package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class RollerIOReal implements RollerIO {
    private final TalonFX motor;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Voltage> voltage;

    private VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);
    private PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);
    private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    
    public RollerIOReal(int motorID, TalonFXConfiguration config) {
        motor = new TalonFX(motorID);

        position = motor.getPosition();
        velocity = motor.getVelocity();
        statorCurrent = motor.getStatorCurrent();
        supplyCurrent = motor.getSupplyCurrent();
        temperature = motor.getDeviceTemp();
        voltage = motor.getMotorVoltage();

        motor.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, statorCurrent, supplyCurrent, temperature, voltage);
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        BaseStatusSignal.refreshAll(position, velocity, statorCurrent, supplyCurrent, temperature, voltage);

        inputs.position = new Rotation2d(position.getValue());
        inputs.velocityRotationsPerSecond = velocity.getValue().in(RotationsPerSecond);
        inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.tempC = temperature.getValueAsDouble();
        inputs.voltage = voltage.getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void setPositionSetpoint(Rotation2d setpoint) {
        motor.setControl(positionVoltage.withPosition(setpoint.getMeasure()));
    }

    @Override
    public void setVelocitySetpoint(double velocityRotationsPerSecond) {
        motor.setControl(velocityVoltage.withVelocity(velocityRotationsPerSecond));
    }

    @Override
    public void resetEncoder(Rotation2d position) {
        motor.setPosition(position.getMeasure());
    }
}
