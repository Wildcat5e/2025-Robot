package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private static final double TOLERANCE = 0.25;
  TalonFX motorOne = new TalonFX(0);
  TalonFX motorTwo = new TalonFX(1);
  State currentState;
  double desiredHeight;

  public enum State {
      NOT_MOVING,
      MOVING_UP,
      MOVING_DOWN;
  }

  
  public Elevator() {
    currentState = State.NOT_MOVING;
  }


  @Override
  public void periodic() {
    switch (currentState) {
      case NOT_MOVING:
        currentState = applyMovementLogic(desiredHeight);
        break;
      case MOVING_UP:
        currentState = applyMovementLogic(desiredHeight);
        break;
      case MOVING_DOWN:
        currentState = applyMovementLogic(desiredHeight);
        break;
    }
  }

  public State applyMovementLogic(double desiredHeight) {
    double currentHeight = motorOne.getPosition().getValueAsDouble();
    double deltaHeight = desiredHeight - currentHeight;

    if (deltaHeight > TOLERANCE) {
      currentState = State.MOVING_UP;
      motorOne.set(0.5);
      motorTwo.set(0.5);
    } else if (deltaHeight < TOLERANCE) {
      currentState = State.MOVING_DOWN;
      motorOne.set(-0.5);
      motorTwo.set(-0.5);
    } else {
      currentState = State.NOT_MOVING;
      motorOne.set(0.0);
      motorTwo.set(0.0);
    }

    return currentState;
  }

  public State setDesiredHeight(double desiredHeight) {
    this.desiredHeight = desiredHeight;
    return applyMovementLogic(desiredHeight);
  }


  public State getCurrentState() {
    return currentState;
  }
}
