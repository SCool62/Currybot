package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure {
  public static enum State {
    IDLE(0, false, ArmState.IDLE, ElevatorState.IDLE),

    // Just balls
    INTAKE_BALL_1(0, false, ArmState.IDLE, ElevatorState.IDLE),
    INTAKE_BALL_2(1, false, ArmState.IDLE, ElevatorState.IDLE),
    REJECT_BALL_1(0, false, ArmState.IDLE, ElevatorState.IDLE),
    REJECT_BALL_2(1, false, ArmState.IDLE, ElevatorState.IDLE),
    INDEX_BALL_1(1, false, ArmState.IDLE, ElevatorState.IDLE),
    INDEX_BALL_2(2, false, ArmState.IDLE, ElevatorState.IDLE),
    READY_BALL_1(1, false, ArmState.IDLE, ElevatorState.IDLE),
    READY_BALL_2(2, false, ArmState.IDLE, ElevatorState.IDLE),
    SHOOT_BALL_1(0, false, ArmState.IDLE, ElevatorState.IDLE),
    SHOOT_BALL_2(1, false, ArmState.IDLE, ElevatorState.IDLE),

    // Just panels
    INTAKE_PANEL(0, false, ArmState.INTAKE_PANEL, ElevatorState.INTAKE_PANEL),
    READY_PANEL(0, true, ArmState.READY_PANEL, ElevatorState.READY_PANEL),
    SCORE_PANEL_LOW(0, false, ArmState.SCORE_PANEL, ElevatorState.SCORE_PANEL_LOW),
    SCORE_PANEL_HIGH(0, false, ArmState.SCORE_PANEL, ElevatorState.SCORE_PANEL_HIGH),

    // Both balls and panels
    INTAKE_BALL_1_WITH_PANEL(0, true, ArmState.READY_PANEL, ElevatorState.READY_PANEL),
    INTAKE_BALL_2_WITH_PANEL(1, true, ArmState.READY_PANEL, ElevatorState.READY_PANEL),
    REJECT_BALL_1_WITH_PANEL(0, true, ArmState.READY_PANEL, ElevatorState.READY_PANEL),
    REJECT_BALL_2_WITH_PANEL(1, true, ArmState.READY_PANEL, ElevatorState.READY_PANEL),
    INDEX_BALL_1_WITH_PANEL(1, true, ArmState.READY_PANEL, ElevatorState.READY_PANEL),
    INDEX_BALL_2_WITH_PANEL(2, true, ArmState.READY_PANEL, ElevatorState.READY_PANEL),
    READY_BALL_1_WITH_PANEL(1, true, ArmState.READY_PANEL, ElevatorState.READY_PANEL),
    READY_BALL_2_WITH_PANEL(2, true, ArmState.READY_PANEL, ElevatorState.READY_PANEL),
    SHOOT_BALL_1_WITH_PANEL(0, true, ArmState.READY_PANEL, ElevatorState.READY_PANEL),
    SHOOT_BALL_2_WITH_PANEL(1, true, ArmState.READY_PANEL, ElevatorState.READY_PANEL),

    INTAKE_PANEL_WITH_BALL_1(1, false, ArmState.INTAKE_PANEL, ElevatorState.INTAKE_PANEL),
    SCORE_PANEL_LOW_WITH_BALL_1(1, false, ArmState.SCORE_PANEL, ElevatorState.SCORE_PANEL_LOW),
    SCORE_PANEL_HIGH_WITH_BALL_1(1, false, ArmState.SCORE_PANEL, ElevatorState.SCORE_PANEL_HIGH),
    INTAKE_PANEL_WITH_BALL_2(2, false, ArmState.INTAKE_PANEL, ElevatorState.INTAKE_PANEL),
    SCORE_PANEL_LOW_WITH_BALL_2(2, false, ArmState.SCORE_PANEL, ElevatorState.SCORE_PANEL_LOW),
    SCORE_PANEL_HIGH_WITH_BALL_2(2, false, ArmState.SCORE_PANEL, ElevatorState.SCORE_PANEL_HIGH),
    ;
    // TODO: ADD MECH SPECIFIC STATES

    private final int numBalls;
    private final boolean hasPanel;
    private final ArmState armState;
    private final ElevatorState elevatorState;

    // Put -1 for unknown amount of balls
    private State(int numBalls, boolean hasPanel, ArmState armState, ElevatorState elevatorState) {
      this.numBalls = numBalls;
      this.hasPanel = hasPanel;
      this.armState = armState;
      this.elevatorState = elevatorState;
    }

    public boolean hasNoBall() {
      return numBalls == 0;
    }

    public boolean hasOneBall() {
      return numBalls == 1;
    }

    public boolean hasTwoBalls() {
      return numBalls == 2;
    }

    public boolean hasPanel() {
      return hasPanel;
    }

    public ArmState getArmState() {
      return armState;
    }

    public ElevatorState getElevatorState() {
      return elevatorState;
    }
  }

  private final ArmSubsystem arm;
  private final IntakeSubsystem intake;
  private final RoutingSubsystem routing;

  @AutoLogOutput(key = "Superstructure/State")
  private State state = State.IDLE;

  @AutoLogOutput(key = "Superstructure/Previous State")
  private State prevState = State.IDLE;

  // Triggers
  private Trigger intakeBallReq;
  private Trigger intakePanelReq;

  private Trigger scoreBallReq;

  private Trigger scorePanelHighReq;
  private Trigger scorePanelLowReq;

  private Trigger intakeBeambreakTrigger;
  private Trigger correctBallColorTrigger;

  private Trigger routingIndexerBeambreakTrigger;

  @AutoLogOutput(key = "Superstructure/At Extension")
  private Trigger atExtensionTrigger;

  public Superstructure(
      CommandXboxController driver,
      CommandXboxController operator,
      ArmSubsystem arm,
      IntakeSubsystem intake,
      RoutingSubsystem routing,
      ElevatorSubsystem elevator) {
    this.arm = arm;
    this.intake = intake;
    this.routing = routing;

    intakeBallReq = driver.leftTrigger();
    intakePanelReq = driver.leftBumper();

    scoreBallReq = driver.rightTrigger();

    intakeBeambreakTrigger = new Trigger(intake::getBeambreakIsDetected);
    correctBallColorTrigger = new Trigger(intake::sensedIsAllianceColor);

    routingIndexerBeambreakTrigger = new Trigger(routing::getCANrangeIsDetected);

    // TODO: ADD REST OF MECHS TO THIS TRIGGER
    atExtensionTrigger = new Trigger(arm::atExtension).and(elevator::atExtension);

    // Call this after setting triggers
    bindTransitions();
  }

  private void bindTransitions() {

    { // Ball states without panels
      bindTransition(State.IDLE, State.INTAKE_BALL_1, intakeBallReq);

      bindTransition(
          State.INTAKE_BALL_1,
          State.INDEX_BALL_1,
          intakeBallReq.negate().and(intakeBeambreakTrigger).and(correctBallColorTrigger));

      bindTransition(
          State.INTAKE_BALL_1,
          State.REJECT_BALL_1,
          correctBallColorTrigger.negate().and(intakeBeambreakTrigger));

      bindTransition(
          State.INTAKE_BALL_2,
          State.INDEX_BALL_2,
          intakeBallReq.negate().and(intakeBeambreakTrigger).and(correctBallColorTrigger));

      bindTransition(
          State.INTAKE_BALL_2,
          State.REJECT_BALL_2,
          correctBallColorTrigger.negate().and(intakeBeambreakTrigger));

      bindTransition(State.REJECT_BALL_1, State.IDLE, intakeBeambreakTrigger.negate().debounce(1));

      bindTransition(
          State.REJECT_BALL_2, State.READY_BALL_1, intakeBeambreakTrigger.negate().debounce(1));

      bindTransition(
          State.INDEX_BALL_1,
          State.READY_BALL_1,
          routingIndexerBeambreakTrigger); // TODO: IS THIS THE RIGHT CONDITION?

      bindTransition(
          State.INDEX_BALL_2,
          State.READY_BALL_2,
          intakeBeambreakTrigger.negate()); // Assume it indexes properly (maybe add a delay)

      bindTransition(State.READY_BALL_1, State.SHOOT_BALL_1, scoreBallReq.and(atExtensionTrigger));

      bindTransition(
          State.SHOOT_BALL_1,
          State.IDLE,
          routingIndexerBeambreakTrigger.negate()); // TODO: Need to check this condition too...

      bindTransition(State.READY_BALL_2, State.SHOOT_BALL_2, scoreBallReq.and(atExtensionTrigger));

      // After it shoots, index the next one
      bindTransition(
          State.SHOOT_BALL_2,
          State.INDEX_BALL_1,
          routingIndexerBeambreakTrigger.negate()); // TODO: CORRECT CONDITION?
    }

    { // Ball states with panels
      bindTransition(State.READY_PANEL, State.INTAKE_BALL_1_WITH_PANEL, intakeBallReq);

      bindTransition(State.READY_BALL_1_WITH_PANEL, State.INTAKE_BALL_2_WITH_PANEL, intakeBallReq);

      bindTransition(
          State.INTAKE_BALL_1_WITH_PANEL,
          State.INDEX_BALL_1_WITH_PANEL,
          intakeBallReq.negate().and(intakeBeambreakTrigger).and(correctBallColorTrigger));

      bindTransition(
          State.INTAKE_BALL_1_WITH_PANEL,
          State.REJECT_BALL_1_WITH_PANEL,
          correctBallColorTrigger.negate().and(intakeBeambreakTrigger));

      bindTransition(
          State.INTAKE_BALL_2_WITH_PANEL,
          State.INDEX_BALL_2_WITH_PANEL,
          intakeBallReq.negate().and(intakeBeambreakTrigger).and(correctBallColorTrigger));

      bindTransition(
          State.INTAKE_BALL_2_WITH_PANEL,
          State.REJECT_BALL_2_WITH_PANEL,
          correctBallColorTrigger.negate().and(intakeBeambreakTrigger));

      bindTransition(
          State.REJECT_BALL_1_WITH_PANEL,
          State.READY_PANEL,
          intakeBeambreakTrigger.negate().debounce(1));

      bindTransition(
          State.REJECT_BALL_2_WITH_PANEL,
          State.READY_BALL_1_WITH_PANEL,
          intakeBeambreakTrigger.negate().debounce(1));

      bindTransition(
          State.INDEX_BALL_1_WITH_PANEL,
          State.READY_BALL_1_WITH_PANEL,
          routingIndexerBeambreakTrigger); // TODO: IS THIS THE RIGHT CONDITION?

      bindTransition(
          State.INDEX_BALL_2_WITH_PANEL,
          State.READY_BALL_2_WITH_PANEL,
          intakeBeambreakTrigger.negate()); // Assume it indexes properly (maybe add a delay)

      bindTransition(
          State.READY_BALL_1_WITH_PANEL,
          State.SHOOT_BALL_1_WITH_PANEL,
          scoreBallReq.and(atExtensionTrigger));

      bindTransition(
          State.SHOOT_BALL_1_WITH_PANEL,
          State.READY_PANEL,
          routingIndexerBeambreakTrigger.negate()); // TODO: Need to check this condition too...

      bindTransition(
          State.READY_BALL_2_WITH_PANEL,
          State.SHOOT_BALL_2_WITH_PANEL,
          scoreBallReq.and(atExtensionTrigger));

      // After it shoots, index the next one
      bindTransition(
          State.SHOOT_BALL_2_WITH_PANEL,
          State.INDEX_BALL_1_WITH_PANEL,
          routingIndexerBeambreakTrigger.negate()); // TODO: CORRECT CONDITION?
    }

    { // Panel states without balls
      bindTransition(State.IDLE, State.INTAKE_PANEL, intakePanelReq);

      bindTransition(State.INTAKE_PANEL, State.READY_PANEL, arm::hasPanel);

      bindTransition(
          State.READY_PANEL, State.SCORE_PANEL_HIGH, scorePanelHighReq.and(atExtensionTrigger));

      bindTransition(
          State.READY_PANEL, State.SCORE_PANEL_LOW, scorePanelLowReq.and(atExtensionTrigger));

      bindTransition(State.SCORE_PANEL_HIGH, State.IDLE, () -> !arm.hasPanel());

      bindTransition(State.SCORE_PANEL_LOW, State.IDLE, () -> !arm.hasPanel());
    }

    { // Panel states with balls

      // ---- WITH 1 BALL ----

      bindTransition(State.READY_BALL_1, State.INTAKE_PANEL_WITH_BALL_1, intakePanelReq);

      bindTransition(State.INTAKE_PANEL_WITH_BALL_1, State.READY_BALL_1_WITH_PANEL, arm::hasPanel);

      bindTransition(
          State.READY_BALL_1_WITH_PANEL,
          State.SCORE_PANEL_HIGH_WITH_BALL_1,
          scorePanelHighReq.and(atExtensionTrigger));

      bindTransition(
          State.READY_BALL_1_WITH_PANEL,
          State.SCORE_PANEL_LOW_WITH_BALL_1,
          scorePanelLowReq.and(atExtensionTrigger));

      bindTransition(State.SCORE_PANEL_HIGH_WITH_BALL_1, State.READY_BALL_1, () -> !arm.hasPanel());

      bindTransition(State.SCORE_PANEL_LOW_WITH_BALL_1, State.READY_BALL_1, () -> !arm.hasPanel());

      // ---- WITH 2 BALLS ----

      bindTransition(State.READY_BALL_2, State.INTAKE_PANEL_WITH_BALL_2, intakePanelReq);

      bindTransition(State.INTAKE_PANEL_WITH_BALL_2, State.READY_BALL_2_WITH_PANEL, arm::hasPanel);

      bindTransition(
          State.READY_BALL_2_WITH_PANEL,
          State.SCORE_PANEL_HIGH_WITH_BALL_2,
          scorePanelHighReq.and(atExtensionTrigger));

      bindTransition(
          State.READY_BALL_2_WITH_PANEL,
          State.SCORE_PANEL_LOW_WITH_BALL_2,
          scorePanelLowReq.and(atExtensionTrigger));

      bindTransition(State.SCORE_PANEL_HIGH_WITH_BALL_2, State.READY_BALL_2, () -> !arm.hasPanel());

      bindTransition(State.SCORE_PANEL_LOW_WITH_BALL_2, State.READY_BALL_2, () -> !arm.hasPanel());
    }
  }

  // Don't need an overload because Triggers are BooleanSuppliers
  private void bindTransition(State from, State to, BooleanSupplier trigger) {
    // When the state is the state we're transitioning from, and the trigger is true, change the
    // state
    new Trigger(() -> state.equals(from)).and(trigger).onTrue(changeStateTo(to));
  }

  private Command changeStateTo(State newState) {
    return Commands.runOnce(
        () -> {
          System.out.println(
              "Changing state from " + state.toString() + " to " + newState.toString());
          this.prevState = this.state;
          this.state = newState;
        });
  }

  public State getState() {
    return state;
  }
}
