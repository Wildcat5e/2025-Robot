package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.LED;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class Elevator extends SubsystemBase {
  private static final double TOLERANCE = 0.0127;
  private static final double POSITION_ZERO = 0.0;
  private static final double LEVEL_TWO = 0.30;
  private static final double LEVEL_THREE = 0.70;
  private static final double GEAR_RATIO = 20.0;
  private static final double SPOOL_DIAMETER_METERS = 0.0199898;
  private static final double SPOOL_CIRCUMFERENCE_METERS = Math.PI * SPOOL_DIAMETER_METERS;
  private final TalonFX motor = new TalonFX(14);
  private final DigitalInput bottomBeamBreak = new DigitalInput(0);
  private final BooleanPublisher bottomBeamBreakPublisher;
  private final DoublePublisher currentHeightPublisher;
  private final DoublePublisher targetHeightPublisher;
  private final StringPublisher currentStatePublisher;
  private final LED led;
  private State currentState;
  private State lastState;
  private double currentHeight;
  private double targetHeight;
  
  private enum State {
    NOT_MOVING,
    MOVING_UP,
    MOVING_DOWN,
    HOLDING_POSITION,
    MANUAL_UP,
    MANUAL_DOWN
  }
    
  public Elevator(LED led) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor.getConfigurator().apply(config);

    currentState = State.NOT_MOVING;
    motor.setPosition(0.0);
    lastState = null;

    NetworkTable elevator = NetworkTableInstance.getDefault().getTable("Elevator");
    bottomBeamBreakPublisher = elevator.getBooleanTopic("Bottom Beam Break").publish();
    currentStatePublisher = elevator.getStringTopic("Current State").publish();
    currentHeightPublisher = elevator.getDoubleTopic("Current Height").publish();
    targetHeightPublisher = elevator.getDoubleTopic("Target Height").publish();
    SmartDashboard.putData(this);

    this.led = led;
  }
  
  @Override
  public void periodic() {
    currentHeight = getCurrentHeight();

    if ((currentState == State.MOVING_DOWN || currentState == State.MANUAL_DOWN) && !bottomBeamBreak.get()) {
      motor.setPosition(0.0);
      setTargetHeight(0.0);
    }

    lastState = currentState;

    handleStateTransition(currentHeight);

    if (currentState != lastState) {
      if (currentState == State.NOT_MOVING) {
        led.setLEDs(LED.BLACK, LED.BLOCK_1[0], LED.BLOCK_1[1]);
      } else if (currentState == State.MOVING_UP) {
        led.setLEDs(LED.GREEN, LED.BLOCK_1[0], LED.BLOCK_1[1]);
      } else if (currentState == State.MOVING_DOWN) {
        led.setLEDs(LED.RED, LED.BLOCK_1[0], LED.BLOCK_1[1]);
      } else if (currentState == State.HOLDING_POSITION) {
        led.setLEDs(LED.YELLOW, LED.BLOCK_1[0], LED.BLOCK_1[1]);
      }
    }

    bottomBeamBreakPublisher.set(bottomBeamBreak.get());
    currentStatePublisher.set(currentState.toString());
    currentHeightPublisher.set(currentHeight);
    targetHeightPublisher.set(targetHeight);

    switch (currentState) {
      case NOT_MOVING:
        motor.set(0.0);
        break;
      case MOVING_UP:
        motor.set(1.0);
        break;
      case MOVING_DOWN:
        motor.set(-0.8);
        break;
      case HOLDING_POSITION:
        motor.set(0.0);
        break;
      case MANUAL_UP:
        motor.set(0.25);
        break;
      case MANUAL_DOWN:
        motor.set(-0.25);
        break;
    }
  }
  
  private void handleStateTransition(double currentHeight) {
    if((currentState == State.MOVING_DOWN || currentState == State.MANUAL_DOWN) && !bottomBeamBreak.get()) {
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
    return run(() -> setTargetHeight(POSITION_ZERO)).until(() -> withinTolerance()).withName("Move to Position Zero");
  }
  
  public Command moveToLevelTwo() {
    return run(() -> setTargetHeight(LEVEL_TWO)).until(() -> withinTolerance()).withName("Move to Level Two");
  }
  
  public Command moveToLevelThree() {
    return run(() -> setTargetHeight(LEVEL_THREE)).until(() -> withinTolerance()).withName("Move to Level Three");
  }

  private boolean withinTolerance() {
    return Math.abs(targetHeight - currentHeight) < TOLERANCE;
  }

  public Command manualUp() {
    return runEnd(() -> currentState = State.MANUAL_UP, () -> stop());
  }

  public Command manualDown() {
    return runEnd(() -> currentState = State.MANUAL_DOWN, () -> stop());
  }

  public void stop() {
    currentState = State.NOT_MOVING;
    setTargetHeight(getCurrentHeight());
  }

  public void configure() {
    if(bottomBeamBreak.get()) {
      setTargetHeight(Double.NEGATIVE_INFINITY);
    }
  }
}