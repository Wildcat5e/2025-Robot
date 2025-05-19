package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
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
  private static final double SPOOL_RADIUS_METERS = 0.0099949;
  private static final double SPOOL_CIRCUMFERENCE_METERS = 2 * Math.PI * SPOOL_RADIUS_METERS;
  private final TalonFX motor = new TalonFX(14);
  private final DigitalInput bottomBeamBreak = new DigitalInput(0);
  private final BooleanPublisher bottomBeamBreakPublisher;
  private final DoublePublisher currentHeightPublisher;
  private final DoublePublisher targetHeightPublisher;
  private final StringPublisher currentStatePublisher;
  private final LED led;
  private State currentState;
  private double currentHeight;
  private double targetHeight;
  
  public enum State {
    NOT_MOVING,
    MOVING_UP,
    MOVING_DOWN
  }
    
  public Elevator(LED led) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor.getConfigurator().apply(config);

    currentState = State.NOT_MOVING;
    motor.setPosition(0.0);

    NetworkTable elevator = NetworkTableInstance.getDefault().getTable("Elevator");
    bottomBeamBreakPublisher = elevator.getBooleanTopic("Bottom Beam Break").publish();
    currentStatePublisher = elevator.getStringTopic("Current State").publish();
    currentHeightPublisher = elevator.getDoubleTopic("Current Height").publish();
    targetHeightPublisher = elevator.getDoubleTopic("Target Height").publish();

    this.led = led;
  }
  
  @Override
  public void periodic() {
    currentHeight = getCurrentHeight();

    if ((currentState == State.MOVING_DOWN) && !bottomBeamBreak.get()) {
      currentState = State.NOT_MOVING;
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
        led.setLEDs(LED.BLACK);
        break;
      case MOVING_UP:
        motor.setVoltage(5.0);
        led.setLEDs(LED.GREEN);
        break;
      case MOVING_DOWN:
        motor.setVoltage(-5.0);
        led.setLEDs(LED.RED);
        break;
    }
  }
  
  private void handleStateTransition(double currentHeight) {
    if (Math.abs(targetHeight - currentHeight) <= TOLERANCE && targetHeight > 0) {
      currentState = State.NOT_MOVING;
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
    return runOnce(() -> setTargetHeight(POSITION_ZERO)).until(() -> withinTolerance());
  }
  
  public Command moveToLevelTwo() {
    return runOnce(() -> setTargetHeight(LEVEL_TWO)).until(() -> withinTolerance());
  }
  
  public Command moveToLevelThree() {
    return runOnce(() -> setTargetHeight(LEVEL_THREE)).until(() -> withinTolerance());
  }

  private boolean withinTolerance() {
    return Math.abs(targetHeight - currentHeight) < TOLERANCE;
  }

  public void configure() {
    if(bottomBeamBreak.get()) {
      setTargetHeight(Double.NEGATIVE_INFINITY);
    }
  }
}