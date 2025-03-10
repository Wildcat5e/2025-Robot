package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;

public class Outtake extends SubsystemBase {
  public static final double TOLERANCE = 0.0;
  private final TalonFX motor = new TalonFX(15);
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
        output = 3.0;
        break;
      case JOGGING_BACKWARD:
        output = -3.0;
        break;
    }

    motor.setVoltage(output);
  }

  public Command outtakeCommand() {
    return runOnce(() -> currentState = State.MOVING_FORWARD);
  }

  public Command jogForwardCommand() {
    return runEnd(() -> currentState = State.JOGGING_FORWARD, () -> currentState = State.NOT_MOVING);
  }

  public Command jogBackwardCommand() {
    return runEnd(() -> currentState = State.JOGGING_BACKWARD, () -> currentState = State.NOT_MOVING);
  }
}