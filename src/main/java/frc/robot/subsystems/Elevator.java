package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private static final double TOLERANCE = 0.0127;
  private static final double POSITION_ZERO = 0.0;
  private static final double LEVEL_TWO = 0.3;
  private static final double LEVEL_THREE = 0.7;
  private static final double GEAR_RATIO = 20.0;
  private static final double SPOOL_DIAMETER_METERS = 0.0199898;
  private static final double SPOOL_CIRCUMFERENCE_METERS = Math.PI * SPOOL_DIAMETER_METERS;
  private final TalonFX motor = new TalonFX(14);
  private final DigitalInput bottomBeamBreak = new DigitalInput(0);
  private final BooleanPublisher bottomBeamBreakPublisher;
  private final DoublePublisher currentHeightPublisher;
  private final DoublePublisher targetHeightPublisher;
  private final StringPublisher currentStatePublisher;
  private State currentState;
  private double currentHeight;
  private double targetHeight;
  
  public enum State {
    NOT_MOVING,
    MOVING_UP,
    MOVING_DOWN,
    HOLDING_POSITION,
    MANUAL_UP,
    MANUAL_DOWN,
    INIT
  }
    
  public Elevator() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor.getConfigurator().apply(config);

    currentState = State.INIT;
    motor.setPosition(0.0);

    NetworkTable elevator = NetworkTableInstance.getDefault().getTable("Elevator");
    bottomBeamBreakPublisher = elevator.getBooleanTopic("BottomBeamBreak").publish();
    currentHeightPublisher = elevator.getDoubleTopic("CurrentHeight").publish();
    targetHeightPublisher = elevator.getDoubleTopic("TargetHeight").publish();
    currentStatePublisher = elevator.getStringTopic("CurrentState").publish();
    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    currentHeight = getCurrentHeight();

    if ((currentState == State.MOVING_DOWN || currentState == State.MANUAL_DOWN || currentState == State.INIT) && !bottomBeamBreak.get()) {
      motor.setPosition(0.0);
      setTargetHeight(0.0);
    }

    handleStateTransition(currentHeight);

    bottomBeamBreakPublisher.set(bottomBeamBreak.get());
    currentStatePublisher.set(currentState.toString());
    currentHeightPublisher.set(currentHeight);
    targetHeightPublisher.set(targetHeight);

    switch (currentState) {
      case NOT_MOVING:
        motor.setVoltage(0.0);
        break;
      case MOVING_UP:
        motor.setVoltage(12.0);
        break;
      case MOVING_DOWN:
        motor.setVoltage(-8.0);
        break;
      case HOLDING_POSITION:
        motor.setVoltage(0.0);
        break;
      case MANUAL_UP:
        motor.setVoltage(3.0);
        break;
      case MANUAL_DOWN:
        motor.setVoltage(-3.0);
        break;
      case INIT:
        motor.setVoltage(-6.0);
        break;
    }
  }
  
  private void handleStateTransition(double currentHeight) {
    if (currentState == State.INIT) {
      if (!bottomBeamBreak.get()) {
        currentState = State.NOT_MOVING;
      }
      return;
    } else if ((currentState == State.MOVING_DOWN || currentState == State.MANUAL_DOWN) && !bottomBeamBreak.get()) {
      currentState = State.NOT_MOVING;
    } else if (currentState == State.MANUAL_UP || currentState == State.MANUAL_DOWN) {
      return;
    } else if (Math.abs(targetHeight - currentHeight) <= TOLERANCE && targetHeight > 0.0) {
      currentState = State.HOLDING_POSITION;
    } else if (targetHeight > currentHeight + TOLERANCE) {
      currentState = State.MOVING_UP;
    } else if (targetHeight < currentHeight - TOLERANCE) {
      currentState = State.MOVING_DOWN;
    }
  }
  
  private void setTargetHeight(double targetHeight) {
    this.targetHeight = targetHeight;
  }
  
  private double getCurrentHeight() {
    return motor.getPosition().getValueAsDouble() / GEAR_RATIO * SPOOL_CIRCUMFERENCE_METERS;
  }

  public Command moveToPositionZero() {
    return new FunctionalCommand(
      () -> setTargetHeight(POSITION_ZERO),
      () -> {},
      (interrupted) -> {},
      () -> withinTolerance(),
      this
    );
  }
  
  public Command moveToLevelTwo() {
    return new FunctionalCommand(
      () -> setTargetHeight(LEVEL_TWO),
      () -> {},
      (interrupted) -> {},
      () -> withinTolerance(),
      this
    );
  }
  
  public Command moveToLevelThree() {
    return new FunctionalCommand(
      () -> setTargetHeight(LEVEL_THREE),
      () -> {},
      (interrupted) -> {},
      () -> withinTolerance(),
      this
    );
  }

  private boolean withinTolerance() {
    return Math.abs(targetHeight - currentHeight) <= TOLERANCE;
  }

  public Command manualUp() {
    return startEnd(() -> currentState = State.MANUAL_UP, () -> stop());
  }

  public Command manualDown() {
    return startEnd(() -> currentState = State.MANUAL_DOWN, () -> stop());
  }

  private void stop() {
    currentState = State.HOLDING_POSITION;
    setTargetHeight(getCurrentHeight());
  }
}