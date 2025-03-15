package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ElevatorBangBangControl extends SubsystemBase implements Elevator {
  public static final double TOLERANCE = 2.0;
  private final TalonFX motor = new TalonFX(14);
  private final DigitalInput beamBreak = new DigitalInput(0);
  private State currentState;
  private double desiredHeight;
  private double currentHeight;
  private double output;
  private BooleanSubscriber beamBreakSubscriber;
  private BooleanPublisher beamBreakPublisher;

  public enum State {
    NOT_MOVING,
    MOVING_UP,
    MOVING_DOWN,
    JOGGING_UP,
    JOGGING_DOWN;
  }

  public ElevatorBangBangControl() {
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
        output = 0.0;
        break;
      case MOVING_UP:
        output = 8.0;
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

    motor.setVoltage(output);
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
    determineNextState();
  }

  private double getCurrentHeight() {
    return motor.getPosition().getValueAsDouble();
  }

  public Command moveToHomePositionCommand() {
    return runOnce(() -> move(Elevator.LEVEL_ZERO_POSITION));
  }

  @Override
  public Command moveToLevelOneCommand() {
    return runOnce(() -> move(Elevator.LEVEL_ONE_POSITION));
  }

  @Override
  public Command moveToLevelTwoCommand() {
    return runOnce(() -> move(Elevator.LEVEL_TWO_POSITION));
  }

  @Override
  public Command moveToLevelThreeCommand() {
    return runOnce(() -> move(Elevator.LEVEL_THREE_POSITION));
  }

  @Override
  public Command jogUpCommand() {
    return runEnd(() -> currentState = State.JOGGING_UP, () -> stop());
  }

  @Override
  public Command jogDownCommand() {
    return runEnd(() -> currentState = State.JOGGING_DOWN, () -> stop());
  }

  @Override
  public void stop() {
    currentState = State.NOT_MOVING;
    desiredHeight = getCurrentHeight();
  }

  @Override
  public boolean isElevatorNotMoving() {
    return currentState == State.NOT_MOVING;
  }

  @Override
  public void updateConfig() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateConfig'");
  }

  @Override
  public Command updateConfigCommand() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateConfigCommand'");
  }

  @Override
  public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sysIdQuasistaticCommand'");
  }

  @Override
  public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sysIdDynamicCommand'");
  }
}