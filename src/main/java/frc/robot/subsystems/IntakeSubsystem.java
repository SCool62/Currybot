package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.canrange.CANRangeIO;
import frc.robot.subsystems.canrange.CANRangeIOInputsAutoLogged;
import frc.robot.subsystems.canrange.CANRangeIOReal;
import frc.robot.subsystems.canrange.CANRangeIOSim;
import frc.robot.subsystems.colorsensor.ColorSensorIO;
import frc.robot.subsystems.colorsensor.ColorSensorIOInputsAutoLogged;
import frc.robot.subsystems.colorsensor.RevColorSensorV3IOReal;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOInputsAutoLogged;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerIOReal;
import frc.robot.subsystems.roller.RollerIOSim;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  public static final double BLUE_THRESHOLD_VALUE = 100;
  public static final double RED_THRESHOLD_VALUE = 100;

  private final PivotIO pivotIO;
  private PivotIOInputsAutoLogged pivotIOInputs = new PivotIOInputsAutoLogged();

  private final RollerIO fourbarRollerIO;
  private RollerIOInputsAutoLogged fourbarRollerIOInputs = new RollerIOInputsAutoLogged();

  private final RollerIO intakeFlywheelIO;
  private RollerIOInputsAutoLogged intakeFlywheelIOInputs = new RollerIOInputsAutoLogged();

  private final CANRangeIO canrangeIO;
  private CANRangeIOInputsAutoLogged canrangeIOInputs = new CANRangeIOInputsAutoLogged();

  private final ColorSensorIO colorSensorIO;
  private ColorSensorIOInputsAutoLogged colorSensorIOInputs = new ColorSensorIOInputsAutoLogged();

  public IntakeSubsystem() {
    CANrangeConfiguration canrangeConfig = new CANrangeConfiguration();
    if (Robot.ROBOT_TYPE.isReal()) {
      TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
      pivotIO = new PivotIOReal(0, pivotConfig);

      // Could set these configs up. Should I or is it not worth it?
      TalonFXConfiguration fourbarRollerConfig = new TalonFXConfiguration();
      fourbarRollerIO = new RollerIOReal(0, fourbarRollerConfig);

      TalonFXConfiguration intakeFlywheelConfig = new TalonFXConfiguration();
      intakeFlywheelIO = new RollerIOReal(0, intakeFlywheelConfig);

      canrangeIO = new CANRangeIOReal(0, canrangeConfig);

      // Onboard I2C
      colorSensorIO = new RevColorSensorV3IOReal(Port.kOnboard);
    } else {
      // TODO: ACTUAL VALUES
      pivotIO =
          new PivotIOSim(
              0.0,
              0.0,
              0.0,
              0.0,
              0.0,
              new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(0.0, 0.0)),
              new ArmFeedforward(0.0, 0.0, 0.0));

      // TODO: ACTUAL VALUES
      fourbarRollerIO =
          new RollerIOSim(
              0,
              0,
              new SimpleMotorFeedforward(0, 0),
              new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)),
              new PIDController(0, 0, 0));

      // TODO: ACTUAL VALUES
      intakeFlywheelIO =
          new RollerIOSim(
              0,
              0,
              new SimpleMotorFeedforward(0, 0),
              new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)),
              new PIDController(0, 0, 0));

      canrangeIO = new CANRangeIOSim(0, canrangeConfig);

      // Not simming color sensor rn
      colorSensorIO = new RevColorSensorV3IOReal(Port.kOnboard);
    }
  }

  @Override
  public void periodic() {
    // Update all the inputs
    fourbarRollerIO.updateInputs(fourbarRollerIOInputs);
    Logger.processInputs("Intake/Fourbar Roller", fourbarRollerIOInputs);

    intakeFlywheelIO.updateInputs(intakeFlywheelIOInputs);
    Logger.processInputs("Intake/Flywheel", intakeFlywheelIOInputs);

    canrangeIO.updateInputs(canrangeIOInputs);
    Logger.processInputs("Intake/CANrange", canrangeIOInputs);

    colorSensorIO.updateInputs(colorSensorIOInputs);
    Logger.processInputs("Intake/Color Sensor", colorSensorIOInputs);
  }

  public Color getSensedColor() {
    // Parses the color from the hex string
    return new Color(colorSensorIOInputs.color);
  }

  public boolean sensedIsAllianceColor() {
    if (DriverStation.getAlliance().isEmpty()) return false;

    Alliance alliance = DriverStation.getAlliance().get();

    Color sensedColor = getSensedColor();
    // I think this would need tuning irl if this robot got built
    return (sensedColor.blue > BLUE_THRESHOLD_VALUE && alliance == Alliance.Blue)
        || (sensedColor.red > RED_THRESHOLD_VALUE && alliance == Alliance.Red);
  }

  public boolean getBeambreakIsDetected() {
    return canrangeIOInputs.isDetected;
  }
}
