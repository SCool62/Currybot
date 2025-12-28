package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  public static final double GEAR_RATIO = 41 / 11; // ~ 3.82/1
  public static final double EXTENSION_TOLERANCE_METERS = Units.inchesToMeters(2); 

  public enum ElevatorState {
    IDLE(0.0),
    INTAKE_PANEL(Inch.of(0.47));

    final double extensionMeters;

    private ElevatorState(double extensionMeters) {
      this.extensionMeters = extensionMeters;
    }

    private ElevatorState(Distance extension) {
      this.extensionMeters = extension.in(Meter);
    }
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  @AutoLogOutput(key = "Elevator/Setpoint")
  private double setpointMeters = 0.0;

  public ElevatorSubsystem() {
    if (Robot.ROBOT_TYPE.isReal()) {
      io = new ElevatorIOReal();
    } else {
      io = new ElevatorIOSim();
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public Command setPositionSetpoint(DoubleSupplier positionSetpoint) {
    return this.run(() -> {
      setpointMeters = positionSetpoint.getAsDouble();
      io.setPositionSetpoint(positionSetpoint.getAsDouble());
    });
  }

  public Command setMotorVoltage(DoubleSupplier voltage) {
    return this.run(() -> io.setVoltage(voltage.getAsDouble()));
  }

  public Command setStateExtension(Supplier<ElevatorState> stateSupplier) {
    return setPositionSetpoint(() -> stateSupplier.get().extensionMeters);
  }

  public boolean atExtension(double setpointMeters) {
    return MathUtil.isNear(setpointMeters, inputs.leaderPositionMeters, EXTENSION_TOLERANCE_METERS);
  }

  @AutoLogOutput(key = "Elevator/At Extension")
  public boolean atExtension() {
    return atExtension(setpointMeters);
  }
}
