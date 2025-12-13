package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private final RollerIO rollerIO;
  private RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();

  private final PivotIO pivotIO;
  private PivotIOInputsAutoLogged pivotIOInputs = new PivotIOInputsAutoLogged();

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
  }

  @Override
  public void periodic() {
    rollerIO.updateInputs(rollerIOInputs);
    Logger.processInputs("Arm/Roller", rollerIOInputs);

    pivotIO.updateInputs(pivotIOInputs);
    Logger.processInputs("Arm/Pivot", pivotIOInputs);
  }


  public Command setPivotSetpoint(Supplier<Rotation2d> position) {
    return this.run(() -> pivotIO.setPositionSetpoint(position.get()));
  }

  public Command setRollerVoltage(DoubleSupplier voltage) {
    return this.run(() -> rollerIO.setVoltage(voltage.getAsDouble()));
  }

  public Command setPivotSetpointAndRollerVoltage(Supplier<Rotation2d> position, DoubleSupplier rollerVoltage) {
    return this.run(() -> {
        rollerIO.setVoltage(rollerVoltage.getAsDouble());
        pivotIO.setPositionSetpoint(position.get());
    });
  }

  public boolean atExtension(Rotation2d setpoint) {
    return pivotIOInputs.position.equals(setpoint);
  }

  @AutoLogOutput(key = "Arm/Pivot/At Extension")
  public boolean atExtension() {
    return atExtension(pivotIO.getSetpoint());
  }
}
