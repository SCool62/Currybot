package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOInputsAutoLogged;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.roller.DoubleMotorRollerIOReal;
import frc.robot.subsystems.roller.DoubleMotorRollerIOSim;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;

public class ShooterSubsystem extends SubsystemBase {

  private final RollerIO rollerIO;
  private RollerIOInputsAutoLogged rollerIOInputs;

  private final PivotIO pivotIO;
  private PivotIOInputsAutoLogged pivotIOInputs;

  public ShooterSubsystem() {
    if (Robot.ROBOT_TYPE.isReal()) {
      // Blank config
      TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
      rollerIO = new DoubleMotorRollerIOReal(0, 0, false, rollerConfig);

      TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
      pivotIO = new PivotIOReal(0, pivotConfig);
    } else {
      // TODO
      rollerIO =
          new DoubleMotorRollerIOSim(
              0.001,
              2.0,
              new SimpleMotorFeedforward(0, 0),
              new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)),
              new PIDController(0, 0, 0));

      pivotIO =
          new PivotIOSim(
              2.0,
              0.035,
              Units.inchesToMeters(8.87),
              // I would figure these out but the mates are broken
              0,
              0,
              new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)),
              new ArmFeedforward(0, 0, 0));
    }
  }

  @Override
  public void periodic() {
      pivotIO.updateInputs(pivotIOInputs);
      Logger.processInputs("Shooter/Pivot", pivotIOInputs);

      rollerIO.updateInputs(rollerIOInputs);
      Logger.processInputs("Shooter/Rollers", rollerIOInputs);
  }

  public Command setPivotAndRoller(Rotation2d angle, double rollerVelocityRotationsPerSecond) {
    return this.run(() -> {
        pivotIO.setPositionSetpoint(angle);
        rollerIO.setVelocitySetpoint(rollerVelocityRotationsPerSecond);
    });
  }
}
