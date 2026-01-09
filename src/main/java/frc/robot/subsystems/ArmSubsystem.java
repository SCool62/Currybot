package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOInputsAutoLogged;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerIOReal;
import frc.robot.subsystems.roller.RollerIOSim;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {

  public static final double PANEL_CURRENT_THRESHOLD = 80.0;

  public enum ArmState {
    // The roller voltage numbers are largly arbitrary, but + is towards the robot and - is away
    // Arm positions estimated from CAD with 0 at straight vertical 
    IDLE(Rotation2d.kZero, 0.0),

    INTAKE_PANEL(Rotation2d.fromDegrees(99.57), 7.0),
    // Could figure out angles for the rest of these but i'm lazy
    READY_PANEL(Rotation2d.kZero, 3.0),
    SCORE_PANEL(Rotation2d.kZero, -5);


    final Rotation2d positionSetpoint;
    final double rollerVoltage;

    private ArmState(Rotation2d positionSetpoint, double rollerVoltage) {
      this.positionSetpoint = positionSetpoint;
      this.rollerVoltage = rollerVoltage;
    }
  }

  private final RollerIO rollerIO;
  private RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();

  private final PivotIO pivotIO;
  private PivotIOInputsAutoLogged pivotIOInputs = new PivotIOInputsAutoLogged();

  private LinearFilter rollerCurrentFilter = LinearFilter.movingAverage(10);

  @AutoLogOutput(key = "Arm/Roller/Current Filter Value")
  private double rollerCurrentFilterValue = 0.0;

  private boolean hasPanel;

  public ArmSubsystem() {
    if (Robot.ROBOT_TYPE.isReal()) {
      // Just factory defualt. Robot isn't real so not sure I need to config it???
      TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

      rollerIO = new RollerIOReal(0, rollerConfig);

      TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

      pivotIO = new PivotIOReal(0, pivotConfig);
    } else {
      // TODO: GET THESE VALUES FROM CAD
      rollerIO =
          new RollerIOSim(
              0,
              0,
              // TODO: TUNE
              new SimpleMotorFeedforward(0, 0),
              new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)),
              new PIDController(0, 0, 0));

      pivotIO =
          new PivotIOSim(
              // TODO: VALUES FROM CAD
              0,
              0,
              0,
              0,
              0,
              new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)),
              new ArmFeedforward(0, 0, 0));
    }

    // Trigger that sets has panel if current threshold is reached
    new Trigger(() -> Math.abs(rollerCurrentFilterValue) > PANEL_CURRENT_THRESHOLD)
        .debounce(0.25)
        .onTrue(Commands.runOnce(() -> hasPanel = true))
        .onFalse(Commands.runOnce(() -> hasPanel = false));
  }

  @Override
  public void periodic() {
    rollerIO.updateInputs(rollerIOInputs);
    Logger.processInputs("Arm/Roller", rollerIOInputs);

    pivotIO.updateInputs(pivotIOInputs);
    Logger.processInputs("Arm/Pivot", pivotIOInputs);

    rollerCurrentFilterValue = rollerCurrentFilter.calculate(rollerIOInputs.statorCurrentAmps);
  }

  public Command setPivotSetpoint(Supplier<Rotation2d> position) {
    return this.run(() -> pivotIO.setPositionSetpoint(position.get()));
  }

  public Command setRollerVoltage(DoubleSupplier voltage) {
    return this.run(() -> rollerIO.setVoltage(voltage.getAsDouble()));
  }

  public Command setPivotSetpointAndRollerVoltage(
      Supplier<Rotation2d> position, DoubleSupplier rollerVoltage) {
    return this.run(
        () -> {
          rollerIO.setVoltage(rollerVoltage.getAsDouble());
          pivotIO.setPositionSetpoint(position.get());
        });
  }

  public Command setStateAngleVoltage(Supplier<ArmState> stateSupplier) {
    return setPivotSetpointAndRollerVoltage(() -> stateSupplier.get().positionSetpoint, () -> stateSupplier.get().rollerVoltage);
  }

  public boolean atExtension(Rotation2d setpoint) {
    return pivotIOInputs.position.equals(setpoint);
  }

  @AutoLogOutput(key = "Arm/Pivot/At Extension")
  public boolean atExtension() {
    return atExtension(pivotIO.getSetpoint());
  }

  @AutoLogOutput(key = "Arm/Has Panel")
  public boolean hasPanel() {
    return hasPanel;
  }
}
