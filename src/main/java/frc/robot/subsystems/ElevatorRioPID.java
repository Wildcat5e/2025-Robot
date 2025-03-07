package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ElevatorRioPID extends SubsystemBase implements Elevator {
  public static final double TOLERANCE = 0.0;
  public static final double MAX_ELEVATOR_SPEED = 12.0;
  private final TalonFX motor = new TalonFX(14);
  private PIDController pidController = new PIDController(0.1, 0.0, 0.0);
  private State currentState;
  private double desiredHeight;
  private double speed;
  private final DoubleSubscriber pConstantSubscriber;
  private final DoubleSubscriber iConstantSubscriber;
  private final DoubleSubscriber dConstantSubscriber;

  public enum State {
    NOT_MOVING,
    MOVING_UP,
    MOVING_DOWN,
    JOGGING_UP,
    JOGGING_DOWN;
  }

  public ElevatorRioPID() {
    currentState = State.NOT_MOVING;
    motor.setPosition(0.0);
    NetworkTable pidConstants = NetworkTableInstance.getDefault().getTable("PID Constants");
    pConstantSubscriber = subscribeToDoubleTopic(pidConstants, "KP", 0.0);
    iConstantSubscriber = subscribeToDoubleTopic(pidConstants, "KI", 0.0);
    dConstantSubscriber = subscribeToDoubleTopic(pidConstants, "KD", 0.0);
    pidController.setTolerance(TOLERANCE);
  }

  /**
   * Subscribe to a double topic. If the topic does not exist,
   * then create the topic and mark it as persistant.
   */
  private DoubleSubscriber subscribeToDoubleTopic(NetworkTable pidConstants, String topicName, double defaultValue) {
    DoubleTopic entry = pidConstants.getDoubleTopic(topicName);
    if (!entry.exists()) {
      System.out.println(String.format("Topic %s does not exist, creating it now.", topicName));
      DoublePublisher publisher = pidConstants.getDoubleTopic(topicName).publish();
      publisher.set(defaultValue);
      entry.setPersistent(true);
      publisher.close();
    } else {
      System.out.println(String.format("Topic %s EXISTS.", topicName));
    }
    return entry.subscribe(defaultValue);
  }

  int counter = 0;
  @Override
  public void periodic() {
    switch (currentState) {
      case NOT_MOVING:
      case MOVING_UP:
      case MOVING_DOWN:
        speed = pidController.calculate(getCurrentHeight(), desiredHeight);
        break;
      case JOGGING_UP:
        speed = 3.0;
        break;
      case JOGGING_DOWN:
        speed = -3.0;
        break;
    }

    double clampedSpeed = Math.max(-MAX_ELEVATOR_SPEED, Math.min(MAX_ELEVATOR_SPEED, speed));

    motor.setVoltage(clampedSpeed);
    determineNextState(desiredHeight);
  }

  private void determineNextState(double desiredHeight) {
    double currentHeight = getCurrentHeight();
    this.desiredHeight = desiredHeight;
    pidController.setSetpoint(desiredHeight);

    if (desiredHeight > currentHeight + TOLERANCE) {
      currentState = State.MOVING_UP;
    } else if (desiredHeight < currentHeight - TOLERANCE) {
      currentState = State.MOVING_DOWN;
    } else {
      currentState = State.NOT_MOVING;
    }
  }

  private double getCurrentHeight() {
    return motor.getPosition().getValueAsDouble();
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

  @Override
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

  private void updateConfig() {
    pidController.setP(pConstantSubscriber.get());
    pidController.setI(iConstantSubscriber.get());
    pidController.setD(dConstantSubscriber.get());

    System.out.println("Kp = " + pConstantSubscriber.get() + " Ki = " + iConstantSubscriber.get() + " Kd = " + dConstantSubscriber.get());
  }

  @Override
  public Command updateConfigCommand() {
    return runOnce(() -> updateConfig());
  }

  @Override
  public boolean isElevatorNotMoving() {
    return currentState == State.NOT_MOVING;
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

  // @Override
  // public boolean isElevatorAtHomePosition() {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'isElevatorAtHomePosition'");
  // }
}