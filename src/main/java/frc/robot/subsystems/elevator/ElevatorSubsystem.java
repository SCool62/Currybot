package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    public static final double GEAR_RATIO = 41/11; // ~ 3.82/1

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public ElevatorSubsystem() {
        this.io = new ElevatorIOSim();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }
    
    public Command setPositionSetpoint(DoubleSupplier positionSetpoint) {
        return this.run(() -> io.setPositionSetpoint(positionSetpoint.getAsDouble()));
    }

    public Command setMotorVoltage(DoubleSupplier voltage) {
        return this.run(() -> io.setVoltage(voltage.getAsDouble()));
    }

}
