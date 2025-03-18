package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class Elevator extends SubsystemBase {
  public static final double TOLERANCE = 5.0;
  public static final double LEVEL_ZERO_POSITION = 0.0;
  public static final double LEVEL_TWO_POSITION = 95.0;
  public static final double LEVEL_THREE_POSITION = 225.0;
  private final TalonFX motor = new TalonFX(14);
  private final DigitalInput beamBreak = new DigitalInput(0);
  private BooleanSubscriber beamBreakSubscriber;
  private BooleanPublisher beamBreakPublisher;
  private State currentState;
  private double desiredHeight;
  private double currentHeight;
  private double volts;
  
  public enum State {
    NOT_MOVING,
    MOVING_UP,
    MOVING_DOWN,
    JOGGING_UP,
    JOGGING_DOWN;
  }
    
  public Elevator() {
    currentState = State.NOT_MOVING;
    motor.setPosition(0.0);
    NetworkTable elevator = NetworkTableInstance.getDefault().getTable("Elevator");
    beamBreakSubscriber = elevator.getBooleanTopic("BeamBreakSubscriber").subscribe(false);
    beamBreakPublisher = elevator.getBooleanTopic("BeamBreakPublisher").publish();
  }
  
  @Override
  public void periodic() {
    currentHeight = getCurrentHeight();
    if ((currentState == State.MOVING_DOWN || currentState == State.JOGGING_DOWN) && !beamBreak.get()) {
      motor.setPosition(0.0);
      currentState = State.NOT_MOVING;
    }
    determineNextState();
    beamBreakPublisher.set(beamBreak.get());
    
    switch (currentState) {
      case NOT_MOVING:
        volts = 0.0;
        break;
      case MOVING_UP:
        volts = 12.0;
        break;
      case MOVING_DOWN:
        volts = -8.0;
        break;
      case JOGGING_UP:
        volts = 3.0;
        break;
      case JOGGING_DOWN:
        volts = -3.0;
        break;
    }
    
    motor.setVoltage(volts);
  }
  
  private void determineNextState() {
    if(currentState == State.JOGGING_UP) {
      return;
    } else if (currentState == State.JOGGING_DOWN) {
      return;
    } else if (desiredHeight > currentHeight + TOLERANCE) {
      currentState = State.MOVING_UP;
    } else if (desiredHeight < currentHeight - TOLERANCE) {
      currentState = State.MOVING_DOWN;
    } else {
      currentState = State.NOT_MOVING;
    }
  }
  
  private void move(double desiredHeight) {
    this.desiredHeight = desiredHeight;
  }
  
  private double getCurrentHeight() {
    return motor.getPosition().getValueAsDouble();
  }
  
  public Command moveToHomePositionCommand() {
    return runOnce(() -> move(LEVEL_ZERO_POSITION));
  }
  
  public Command moveToLevelTwoCommand() {
    return runOnce(() -> move(LEVEL_TWO_POSITION));
  }
  
  public Command moveToLevelThreeCommand() {
    return runOnce(() -> move(LEVEL_THREE_POSITION));
  }

  public Command jogUpCommand() {
    return runEnd(() -> currentState = State.JOGGING_UP, () -> stop());
  }

  public Command jogDownCommand() {
    return runEnd(() -> currentState = State.JOGGING_DOWN, () -> stop());
  }

  private void stop() {
    currentState = State.NOT_MOVING;
    desiredHeight = getCurrentHeight();
  }

  public boolean isElevatorNotMoving() {
    return currentState == State.NOT_MOVING;
  }
}