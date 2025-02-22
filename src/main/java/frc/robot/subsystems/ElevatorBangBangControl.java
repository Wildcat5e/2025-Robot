package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorBangBangControl extends SubsystemBase implements Elevator {
  public static final double TOLERANCE = 2.0;
  private final TalonFX motorOne = new TalonFX(13);
  private final TalonFX motorTwo = new TalonFX(14);
  private final Follower follower = new Follower(13, true);
  private State currentState;
  private double desiredHeight;
  private double output;

  public enum State {
    NOT_MOVING,
    MOVING_UP,
    MOVING_DOWN,
    JOGGING_UP,
    JOGGING_DOWN;
  }

  public ElevatorBangBangControl() {
    currentState = State.NOT_MOVING;
    motorTwo.setControl(follower);
    motorOne.setPosition(0.0);
  }

  @Override
  public void periodic() {
    switch (currentState) {
      case NOT_MOVING:
        output = 0.0;
        break;
      case MOVING_UP:
        output = 6.0;
        break;
      case MOVING_DOWN:
        output = -6.0;
        break;
      case JOGGING_UP:
        output = 1.0;
        break;
      case JOGGING_DOWN:
        output = -1.0;
        break;
    }

    motorOne.setVoltage(output);
    determineNextState(desiredHeight);
  }

  private void determineNextState(double desiredHeight) {
    double currentHeight = getCurrentHeight();
    this.desiredHeight = desiredHeight;

    if (desiredHeight > currentHeight + TOLERANCE) {
      currentState = State.MOVING_UP;
    } else if (desiredHeight < currentHeight - TOLERANCE) {
      currentState = State.MOVING_DOWN;
    } else {
      currentState = State.NOT_MOVING;
    }
  }

  private double getCurrentHeight() {
    return motorOne.getPosition().getValueAsDouble();
  }

  private void stop() {
    currentState = State.NOT_MOVING;
    desiredHeight = getCurrentHeight();
  }

  @Override
  public Command jogUpCommand() {
    return runEnd(() -> currentState = State.JOGGING_UP, () -> stop());
  }

  @Override
  public Command jogDownCommand() {
    return runEnd(() -> currentState = State.JOGGING_DOWN, () -> stop());
  }

  public Command moveToHomePositionCommand() {
    return runOnce(() -> determineNextState(Elevator.LEVEL_ZERO_POSITION * Elevator.ENCODER_TICS_PER_INCH));
  }

  @Override
  public Command moveToLevelOneCommand() {
    return runOnce(() -> determineNextState(Elevator.LEVEL_ONE_POSITION * Elevator.ENCODER_TICS_PER_INCH));
  }

  @Override
  public Command moveToLevelTwoCommand() {
    return runOnce(() -> determineNextState(Elevator.LEVEL_TWO_POSITION * Elevator.ENCODER_TICS_PER_INCH));
  }

  @Override
  public Command moveToLevelThreeCommand() {
    return runOnce(() -> determineNextState(Elevator.LEVEL_THREE_POSITION * Elevator.ENCODER_TICS_PER_INCH));
  }

  @Override
  public Command moveToLevelFourCommand() {
    return runOnce(() -> determineNextState(Elevator.LEVEL_FOUR_POSITION * Elevator.ENCODER_TICS_PER_INCH));
  }

  @Override
  public Command updateConfigCommand() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateConfigCommand'");
  }

  public boolean isElevatorNotMoving() {
    return currentState == State.NOT_MOVING;
  }
}