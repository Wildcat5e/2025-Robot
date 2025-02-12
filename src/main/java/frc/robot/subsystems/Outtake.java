package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj2.command.Command;

public class Outtake extends SubsystemBase {
  private final TalonFX leftMotor = new TalonFX(2);
  private final TalonFX rightMotor = new TalonFX(3);
  private final Follower follower = new Follower(2, true);
  private State currentState;

  public enum State {
    NOT_MOVING,
    MOVING_FORWARD,
    MOVING_BACKWARD,
    JOGGING_FORWARD,
    JOGGING_BACKWARD;
  }

  public Outtake() {
    currentState = State.NOT_MOVING;
    rightMotor.setControl(follower);
  }

  @Override
  public void periodic() {
    double output = 0.0;

    switch (currentState) {
      case NOT_MOVING:
        output = 0.0;
        break;
      case MOVING_FORWARD:
        output = 6.0;
        break;
      case MOVING_BACKWARD:
        output = -6.0;
        break;
      case JOGGING_FORWARD:
        output = 1.0;
        break;
      case JOGGING_BACKWARD:
        output = -1.0;
        break;
    }

    leftMotor.setVoltage(output);
    determineNextState();
  }

  private void determineNextState() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'determineNextState'");
  }

  public Command outtakeCommand() {
    return runEnd(() -> currentState = State.MOVING_FORWARD, () -> currentState = State.NOT_MOVING);
  }
}
