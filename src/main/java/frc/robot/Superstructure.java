package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure {
  public static enum State {
    IDLE(0, false),

    // Just balls
    INTAKE_BALL_1(0, false),
    INTAKE_BALL_2(1, false),
    REJECT_BALL_1(0, false),
    REJECT_BALL_2(1, false),
    INDEX_BALL_1(1, false),
    INDEX_BALL_2(2, false),
    READY_BALL_1(1, false),
    READY_BALL_2(2, false),
    SHOOT_BALL_1(0, false),
    SHOOT_BALL_2(1, false),

    // Just panels
    INTAKE_PANEL(0, false),
    READY_PANEL(0, true),
    SCORE_PANEL_LOW(0, false),
    SCORE_PANEL_HIGH(0, false),

    // Both balls and panels
    INTAKE_BALL_1_WITH_PANEL(0, true),
    INTAKE_BALL_2_WITH_PANEL(1, true),
    REJECT_BALL_1_WITH_PANEL(0, true),
    REJECT_BALL_2_WITH_PANEL(1, true),
    INDEX_BALL_1_WITH_PANEL(1, true),
    INDEX_BALL_2_WITH_PANEL(2, true),
    READY_BALL_1_WITH_PANEL(1, true),
    READY_BALL_2_WITH_PANEL(2, true),
    SHOOT_BALL_1_WITH_PANEL(0, true),
    SHOOT_BALL_2_WITH_PANEL(1, true),

    INTAKE_PANEL_WITH_BALL_1(1, false),
    SCORE_PANEL_LOW_WITH_BALL_1(1, false),
    SCORE_PANEL_HIGH_WITH_BALL_1(1, false),
    INTAKE_PANEL_WITH_BALL_2(2, false),
    SCORE_PANEL_LOW_WITH_BALL_2(2, false),
    SCORE_PANEL_HIGH_WITH_BALL_2(2, false),
    ;
    // TODO: ADD MECH SPECIFIC STATES

    private final int numBalls;
    private final boolean hasPanel;

    // Put -1 for unknown amount of balls
    private State(int numBalls, boolean hasPanel) {
      this.numBalls = numBalls;
      this.hasPanel = hasPanel;
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

  public Superstructure(
      CommandXboxController driver,
      CommandXboxController operator,
      ArmSubsystem arm,
      IntakeSubsystem intake,
      RoutingSubsystem routing) {
    this.arm = arm;
    this.intake = intake;
    this.routing = routing;

    intakeBallReq = driver.leftTrigger();
    intakePanelReq = driver.leftBumper();

    scoreBallReq = driver.rightTrigger();

    intakeBeambreakTrigger = new Trigger(intake::getBeambreakIsDetected);
    correctBallColorTrigger = new Trigger(intake::sensedIsAllianceColor);

    routingIndexerBeambreakTrigger = new Trigger(routing::getCANrangeIsDetected);

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

      bindTransition(State.READY_BALL_1, State.SHOOT_BALL_1, scoreBallReq);

      bindTransition(
          State.SHOOT_BALL_1,
          State.IDLE,
          routingIndexerBeambreakTrigger.negate()); // TODO: Need to check this condition too...

      bindTransition(State.READY_BALL_2, State.SHOOT_BALL_2, scoreBallReq);

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

      bindTransition(State.READY_BALL_1_WITH_PANEL, State.SHOOT_BALL_1_WITH_PANEL, scoreBallReq);

      bindTransition(
          State.SHOOT_BALL_1_WITH_PANEL,
          State.READY_PANEL,
          routingIndexerBeambreakTrigger.negate()); // TODO: Need to check this condition too...

      bindTransition(State.READY_BALL_2_WITH_PANEL, State.SHOOT_BALL_2_WITH_PANEL, scoreBallReq);

      // After it shoots, index the next one
      bindTransition(
          State.SHOOT_BALL_2_WITH_PANEL,
          State.INDEX_BALL_1_WITH_PANEL,
          routingIndexerBeambreakTrigger.negate()); // TODO: CORRECT CONDITION?
    }

    { // Panel states without balls
      bindTransition(State.IDLE, State.INTAKE_PANEL, intakePanelReq);

      bindTransition(State.INTAKE_PANEL, State.READY_PANEL, arm::hasPanel);

      bindTransition(State.READY_PANEL, State.SCORE_PANEL_HIGH, scorePanelHighReq);

      bindTransition(State.READY_PANEL, State.SCORE_PANEL_LOW, scorePanelLowReq);

      bindTransition(State.SCORE_PANEL_HIGH, State.IDLE, () -> !arm.hasPanel());

      bindTransition(State.SCORE_PANEL_LOW, State.IDLE, () -> !arm.hasPanel());
    }

    { // Panel states with balls

      // ---- WITH 1 BALL ----

      bindTransition(State.READY_BALL_1, State.INTAKE_PANEL_WITH_BALL_1, intakePanelReq);

      bindTransition(State.INTAKE_PANEL_WITH_BALL_1, State.READY_BALL_1_WITH_PANEL, arm::hasPanel);

      bindTransition(
          State.READY_BALL_1_WITH_PANEL, State.SCORE_PANEL_HIGH_WITH_BALL_1, scorePanelHighReq);

      bindTransition(
          State.READY_BALL_1_WITH_PANEL, State.SCORE_PANEL_LOW_WITH_BALL_1, scorePanelLowReq);

      bindTransition(State.SCORE_PANEL_HIGH_WITH_BALL_1, State.READY_BALL_1, () -> !arm.hasPanel());

      bindTransition(State.SCORE_PANEL_LOW_WITH_BALL_1, State.READY_BALL_1, () -> !arm.hasPanel());

      // ---- WITH 2 BALLS ----

      bindTransition(State.READY_BALL_2, State.INTAKE_PANEL_WITH_BALL_2, intakePanelReq);

      bindTransition(State.INTAKE_PANEL_WITH_BALL_2, State.READY_BALL_2_WITH_PANEL, arm::hasPanel);

      bindTransition(
          State.READY_BALL_2_WITH_PANEL, State.SCORE_PANEL_HIGH_WITH_BALL_2, scorePanelHighReq);

      bindTransition(
          State.READY_BALL_2_WITH_PANEL, State.SCORE_PANEL_LOW_WITH_BALL_2, scorePanelLowReq);

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
}
