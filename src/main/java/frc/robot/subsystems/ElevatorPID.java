package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// import com.ctre.phoenix6.controls.Follower;

public class ElevatorPID extends SubsystemBase implements Elevator {
  public static final double TOLERANCE = 0.0;
  public static final double MAX_ELEVATOR_SPEED = 12.0;
  private final TalonFX motorOne = new TalonFX(0);
  // private final TalonFX motorTwo = new TalonFX(1);
  // private final Follower follower = new Follower(0, true);
  private PIDController pidController = new PIDController(0.0, 0.0, 0.0);
  private State currentState;
  private double desiredHeight;
  private double speed;
  private double homeHeight;
  private NetworkTable pidConstants = NetworkTableInstance.getDefault().getTable("PID Constants");
  NetworkTableEntry KP = pidConstants.getEntry("KP");
  NetworkTableEntry KI = pidConstants.getEntry("KI");
  NetworkTableEntry KD = pidConstants.getEntry("KD");

  public enum State {
    NOT_MOVING,
    MOVING_UP,
    MOVING_DOWN,
    JOGGING_UP,
    JOGGING_DOWN;
  }

  public ElevatorPID() {
    currentState = State.NOT_MOVING;
    // motorTwo.setControl(follower);
    motorOne.setPosition(0.0);
    KP.setDouble(0.0);
    KI.setDouble(0.0);
    KD.setDouble(0.0);
    System.out.println("At Startup KP: " + pidController.getP() + " KI: " + pidController.getI() + " KD: " + pidController.getD());
    pidController.setTolerance(TOLERANCE);
    setCurrentPositionAsHome();
  }

  private void setCurrentPositionAsHome() {
    homeHeight = getCurrentHeight();
    System.out.println("Home is currently at = " + homeHeight);
  }

  public Command setCurrentPositionAsHomeCommand() {
    return runOnce(() -> setCurrentPositionAsHome());
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
        speed = 1.0;
        break;
      case JOGGING_DOWN:
        speed = -1.0;
        break;
    }

    double clampedSpeed = Math.max(-MAX_ELEVATOR_SPEED, Math.min(MAX_ELEVATOR_SPEED, speed));

    motorOne.setVoltage(clampedSpeed);
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
    return motorOne.getPosition().getValueAsDouble();
  }

  private void stop() {
    desiredHeight = getCurrentHeight();
    currentState = State.NOT_MOVING;
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
  public Command moveToPositionZeroCommand() {
    return runOnce(
        () -> determineNextState(Elevator.LEVEL_ZERO_POSITION * Elevator.ENCODER_TICS_PER_INCH + homeHeight));
  }

  @Override
  public Command moveToLevelOneCommand() {
    return runOnce(() -> determineNextState(Elevator.LEVEL_ONE_POSITION * Elevator.ENCODER_TICS_PER_INCH + homeHeight));
  }

  @Override
  public Command moveToLevelTwoCommand() {
    return runOnce(() -> determineNextState(Elevator.LEVEL_TWO_POSITION * Elevator.ENCODER_TICS_PER_INCH + homeHeight));
  }

  @Override
  public Command moveToLevelThreeCommand() {
    return runOnce(
        () -> determineNextState(Elevator.LEVEL_THREE_POSITION * Elevator.ENCODER_TICS_PER_INCH + homeHeight));
  }

  @Override
  public Command moveToLevelFourCommand() {
    return runOnce(
        () -> determineNextState(Elevator.LEVEL_FOUR_POSITION * Elevator.ENCODER_TICS_PER_INCH + homeHeight));
  }

  private void updateConfig() {
    pidController.setP(KP.getDouble(0.0));
    pidController.setI(KI.getDouble(0.0));
    pidController.setD(KD.getDouble(0.0));
    System.out.println("At updateConfig KP: " + pidController.getP() + " KI: " + pidController.getI() + " KD: " + pidController.getD());
    System.out.println("Network Table KP: " + KP.getDouble(0.0) + " KI: " + KI.getDouble(0.0) + " KD: " + KD.getDouble(0.0));
  }

  public Command updateConfigCommand() {
    return runOnce(() -> updateConfig());
  }
}