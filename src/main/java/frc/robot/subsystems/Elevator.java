package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class Elevator extends SubsystemBase {
  public static final double TOLERANCE = 5;
  public static final double POSITION_ZERO = 0;
  public static final double LEVEL_TWO_POSITION = 100;
  public static final double LEVEL_THREE_POSITION = 230;
  private final TalonFX motor = new TalonFX(14);
  private final DigitalInput bottomBeamBreak = new DigitalInput(0);
  private State currentState;
  private BooleanPublisher bottomBeamBreakPublisher;
  private StringPublisher currentStatePublisher;
  private double desiredHeight;
  private double currentHeight;
  private double volts;
  
  public enum State {
    NOT_MOVING,
    MOVING_UP,
    MOVING_DOWN,
    MANUAL_UP,
    MANUAL_DOWN
  }
    
  public Elevator() {
    currentState = State.NOT_MOVING;
    motor.setPosition(0);
    NetworkTable elevator = NetworkTableInstance.getDefault().getTable("Elevator");
    bottomBeamBreakPublisher = elevator.getBooleanTopic("Bottom Beam Break").publish();
    currentStatePublisher = elevator.getStringTopic("Current State").publish();
  }
  
  @Override
  public void periodic() {
    currentHeight = getCurrentHeight();
    if ((currentState == State.MOVING_DOWN || currentState == State.MANUAL_DOWN) && !bottomBeamBreak.get()) {
      currentState = State.NOT_MOVING;
      motor.setPosition(0);
    }
    determineNextState();
    bottomBeamBreakPublisher.set(bottomBeamBreak.get());
    currentStatePublisher.set(currentState.toString());
    
    switch (currentState) {
      case NOT_MOVING:
        volts = 0;
        break;
      case MOVING_UP:
        volts = 12;
        break;
      case MOVING_DOWN:
        volts = -8;
        break;
      case MANUAL_UP:
        volts = 3;
        break;
      case MANUAL_DOWN:
        volts = -3;
        break;
    }
    
    motor.setVoltage(volts);
  }
  
  private void determineNextState() {
    if (currentState == State.MANUAL_UP || currentState == State.MANUAL_DOWN) {
      return;
    } else if (Math.abs(currentHeight - desiredHeight) <= TOLERANCE && desiredHeight > 0) {
      currentState = State.NOT_MOVING;
    } else if (desiredHeight > currentHeight + TOLERANCE) {
      currentState = State.MOVING_UP;
    } else if (desiredHeight < currentHeight - TOLERANCE) {
      currentState = State.MOVING_DOWN;
    }
  }
  
  private void move(double desiredHeight) {
    this.desiredHeight = desiredHeight;
  }
  
  private double getCurrentHeight() {
    return motor.getPosition().getValueAsDouble();
  }
  
  public Command moveToPositionZeroCommand() {
    return runOnce(() -> move(POSITION_ZERO));
  }
  
  public Command moveToLevelTwoCommand() {
    return runOnce(() -> move(LEVEL_TWO_POSITION));
  }
  
  public Command moveToLevelThreeCommand() {
    return runOnce(() -> move(LEVEL_THREE_POSITION));
  }

  public Command manualUpCommand() {
    return runEnd(() -> currentState = State.MANUAL_UP, () -> stop());
  }

  public Command manualDownCommand() {
    return runEnd(() -> currentState = State.MANUAL_DOWN, () -> stop());
  }

  private void stop() {
    currentState = State.NOT_MOVING;
    desiredHeight = getCurrentHeight();
  }

  public boolean isElevatorNotMoving() {
    return currentState == State.NOT_MOVING;
  }
}