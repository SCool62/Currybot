package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure {
  public static enum State {
    IDLE(0),

    // Just balls
    INTAKE_BALL(-1),
    REJECT_BALL_1(0),
    REJECT_BALL_2(1),
    INDEX_BALL_1(1),
    INDEX_BALL_2(2),
    READY_BALL_1(1),
    READY_BALL_2(2),
    SHOOT_BALL_1(0),
    SHOOT_BALL_2(1),

    // Just panels
    INAKE_PANEL(0),
    READY_PANEL(0),
    SCORE_PANEL_LOW(0),
    SCORE_PANEL_HIGH(0),

    // Both balls and panels
    INTAKE_BALL_WITH_PANEL(-1),
    REJECT_BALL_1_WITH_PANEL(0),
    REJECT_BALL_2_WITH_PANEL(1),
    INDEX_BALL_1_WITH_PANEL(1),
    INDEX_BALL_2_WITH_PANEL(2),
    READY_BALL_1_WITH_PANEL(1),
    READY_BALL_2_WITH_PANEL(2),
    SHOOT_BALL_1_WITH_PANEL(0),
    SHOOT_BALL_2_WITH_PANEL(1),

    INAKE_PANEL_WITH_BALL_1(1),
    READY_PANEL_WITH_BALL_1(1),
    SCORE_PANEL_LOW_WITH_BALL_1(1),
    SCORE_PANEL_HIGH_WITH_BALL_1(1),
    INAKE_PANEL_WITH_BALL_2(2),
    READY_PANEL_WITH_BALL_2(2),
    SCORE_PANEL_LOW_WITH_BALL_2(2),
    SCORE_PANEL_HIGH_WITH_BALL_2(2),
    ;
    // TODO: ADD MECH SPECIFIC STATES

    private final int numBalls;

    // Put -1 for unknown amount of balls
    private State(int numBalls) {
      this.numBalls = numBalls;
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
  }

  @AutoLogOutput(key = "Superstructure/State")
  private State state = State.IDLE;

  @AutoLogOutput(key = "Superstructure/Previous State")
  private State prevState = State.IDLE;

  // Triggers
  // TODO: SET THESE
  private Trigger intakeBallReq;
  private Trigger intakePanelReq;
  private Trigger scoreBallReq;
  private Trigger scorePanelReq;

  private Trigger intakeBeambreakTrigger;
  private Trigger correctBallColorTrigger;

  private Trigger shooterBeambreakTrigger;

  public Superstructure(CommandXboxController driver, CommandXboxController operator) {
    intakeBallReq = driver.leftTrigger();
    intakePanelReq = driver.leftBumper();

    scoreBallReq = driver.rightTrigger();
  }

  private void bindTransition(State from, State to, Trigger trigger) {
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
