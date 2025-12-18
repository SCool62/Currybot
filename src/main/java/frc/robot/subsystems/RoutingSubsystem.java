package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.canrange.CANRangeIO;
import frc.robot.subsystems.canrange.CANRangeIOInputsAutoLogged;
import frc.robot.subsystems.canrange.CANRangeIOReal;
import frc.robot.subsystems.canrange.CANRangeIOSim;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerIOReal;
import frc.robot.subsystems.roller.RollerIOSim;

// The routing has 2 motors. One controls all but one set of rollers
// The second controls the last pair of rollers which index into the shooter
public class RoutingSubsystem extends SubsystemBase {
    
    // The main rollers from the intake until the last pair
    private final RollerIO primaryRoutingRollers;
    private RollerIOInputsAutoLogged primaryRoutingRollerInputs = new RollerIOInputsAutoLogged();

    private final RollerIO shooterIndexRollers;
    private RollerIOInputsAutoLogged shooterIndexRollerInputs = new RollerIOInputsAutoLogged();

    // This canrange isn't actually in the cad but it would be stupid for this robot not to have it so we pretend 
    private final CANRangeIO canRangeIO;
    private CANRangeIOInputsAutoLogged canRangeIOInputs = new CANRangeIOInputsAutoLogged();

    public RoutingSubsystem() {
        CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();

        if (Robot.ROBOT_TYPE.isReal()) {
            TalonFXConfiguration primaryRollerConfig = new TalonFXConfiguration();
            primaryRoutingRollers = new RollerIOReal(0, primaryRollerConfig);

            TalonFXConfiguration shooterIndexRollerConfig = new TalonFXConfiguration();
            shooterIndexRollers = new RollerIOReal(0, shooterIndexRollerConfig);

            canRangeIO = new CANRangeIOReal(0, canRangeConfig);
        } else {
            // TODO: SET THESE VALUES
            primaryRoutingRollers = new RollerIOSim(
                0, 
                0, 
                new SimpleMotorFeedforward(0, 0), 
                new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)), 
                new PIDController(0, 0, 0)
            );

            shooterIndexRollers = new RollerIOSim(
                0, 
                0, 
                new SimpleMotorFeedforward(0, 0), 
                new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)), 
                new PIDController(0, 0, 0)
            );

            canRangeIO = new CANRangeIOSim(0, null);
        }
    }

    @Override
    public void periodic() {
        primaryRoutingRollers.updateInputs(primaryRoutingRollerInputs);
        Logger.processInputs("Routing/Primary Rollers", primaryRoutingRollerInputs);

        shooterIndexRollers.updateInputs(shooterIndexRollerInputs);
        Logger.processInputs("Routing/Shooter Indexing Rollers", shooterIndexRollerInputs);

        canRangeIO.updateInputs(canRangeIOInputs);
        Logger.processInputs("Routing/CANrange", canRangeIOInputs);
    }
}
