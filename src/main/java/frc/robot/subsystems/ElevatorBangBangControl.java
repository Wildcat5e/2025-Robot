package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ElevatorBangBangControl extends SubsystemBase implements Elevator {
  public static final double TOLERANCE = 0.25;
  private final TalonFX motor = new TalonFX(14);
  private final DigitalInput bottomLimiter = new DigitalInput(0);
  private final DigitalInput topLimiter = new DigitalInput(1);
  private State currentState;
  private double desiredHeight;
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

  int counter = 0;
  @Override
  public void periodic() {
    if (counter++ % 250 == 0) {
      System.out.println("current state: " + currentState);
    }
    beamBreakPublisher.set(bottomLimiter.get());
    // enforceBeamBreakLimits();
    // determineNextState(desiredHeight);
    switch (currentState) {
      case NOT_MOVING:
        output = 0.0;
        break;
      case MOVING_UP:
        output = 2.5;
        break;
      case MOVING_DOWN:
        output = -2.0;
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

  private void enforceBeamBreakLimits() {
    if (currentState == State.MOVING_UP && topLimiter.get() ||
        currentState == State.MOVING_DOWN && bottomLimiter.get()) {
      motor.setPosition(0.0);
      currentState = State.NOT_MOVING;
    }
  }

  private double getCurrentHeight() {
    return motor.getPosition().getValueAsDouble();
  }

  public Command moveToHomePositionCommand() {
    return runOnce(() -> determineNextState(Elevator.LEVEL_ZERO_POSITION));
  }

  @Override
  public Command moveToLevelOneCommand() {
    return runOnce(() -> determineNextState(Elevator.LEVEL_ONE_POSITION));
  }

  @Override
  public Command moveToLevelTwoCommand() {
    return runOnce(() -> determineNextState(Elevator.LEVEL_TWO_POSITION));
  }

  @Override
  public Command moveToLevelThreeCommand() {
    return runOnce(() -> determineNextState(Elevator.LEVEL_THREE_POSITION));
  }

  @Override
  public Command jogUpCommand() {
    return runEnd(() -> currentState = State.JOGGING_UP, () -> stop());
  }

  @Override
  public Command jogDownCommand() {
    return runEnd(() -> currentState = State.JOGGING_DOWN, () -> stop());
  }

  private void stop() {
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