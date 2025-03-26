package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.wpilibj2.command.Command;

public class Outtake extends SubsystemBase {
  private final TalonFX motor = new TalonFX(15);
  private final DigitalInput startBeamBreak = new DigitalInput(1);
  private final DigitalInput endBeamBreak = new DigitalInput(2);
  private BooleanPublisher startBeamBreakPublisher;
  private BooleanPublisher endBeamBreakPublisher;
  private StringPublisher currentStatePublisher;
  private boolean shootCommand = false;
  private State currentState;
  private double volts;

  public enum State {
    IDLE,
    LOADING,
    SHOOTING,
    MANUAL_FORWARD,
    MANUAL_BACKWARD
  }

  public Outtake() {
    currentState = State.IDLE;
    NetworkTable outtake = NetworkTableInstance.getDefault().getTable("Outtake");
    startBeamBreakPublisher = outtake.getBooleanTopic("Start Beam Break").publish();
    endBeamBreakPublisher = outtake.getBooleanTopic("End Beam Break").publish();
    currentStatePublisher = outtake.getStringTopic("Current State").publish();
  }

  @Override
  public void periodic() {
    determineNextState();
    startBeamBreakPublisher.set(startBeamBreak.get());
    endBeamBreakPublisher.set(endBeamBreak.get());
    currentStatePublisher.set(currentState.toString());

    switch (currentState) {
      case IDLE:
        volts = 0.0;
        break;
      case LOADING:
        volts = 1.5;
        break;
      case SHOOTING:
        volts = 3.0;
        break;
      case MANUAL_FORWARD:
        volts = 3.0;
        break;
      case MANUAL_BACKWARD:
        volts = -3.0;
        break;
    }

    motor.setVoltage(volts);
  }

  private void determineNextState() {
    if(currentState == State.MANUAL_FORWARD || currentState == State.MANUAL_BACKWARD) {
      return;
    } else if (currentState == State.IDLE && !startBeamBreak.get()) {
      currentState = State.LOADING;
    } else if (currentState == State.LOADING && startBeamBreak.get()) {
      currentState = State.IDLE;
    } else if (shootCommand && !endBeamBreak.get()) {
      currentState = State.SHOOTING;
    } else if (shootCommand && endBeamBreak.get()) {
      currentState = State.IDLE;
      shootCommand = false;
    }
  }

  public Command shootCommand() {
    return runOnce(() -> shootCommand = true);
  }

  public Command manualForwardCommand() {
    return runEnd(() -> currentState = State.MANUAL_FORWARD, () -> stop());
  }

  public Command manualBackwardCommand() {
    return runEnd(() -> currentState = State.MANUAL_BACKWARD, () -> stop());
  }

  private void stop() {
    currentState = State.IDLE;
  }
}