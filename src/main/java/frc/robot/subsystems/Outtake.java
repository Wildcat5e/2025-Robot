package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.Command;

public class Outtake extends SubsystemBase {
  private final TalonFX motor = new TalonFX(15);
  private final DigitalInput beamBreak = new DigitalInput(1);
  private BooleanSubscriber beamBreakSubscriber;
  private BooleanPublisher beamBreakPublisher;
  private State currentState;
  private double volts;

  public enum State {
    IDLE,
    LOADING,
    SHOOTING,
    JOGGING_FORWARD,
    JOGGING_BACKWARD;
  }

  public Outtake() {
    currentState = State.IDLE;
    NetworkTable outtake = NetworkTableInstance.getDefault().getTable("Outtake");
    beamBreakSubscriber = outtake.getBooleanTopic("beamBreakSubscriber").subscribe(false);
    beamBreakPublisher = outtake.getBooleanTopic("beamBreakPublisher").publish();
  }

  @Override
  public void periodic() {
    determineNextState();
    beamBreakPublisher.set(beamBreak.get());

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
      case JOGGING_FORWARD:
        volts = 3.0;
        break;
      case JOGGING_BACKWARD:
        volts = -3.0;
        break;
    }

    motor.setVoltage(volts);
  }

  private void determineNextState() {
    if(currentState == State.JOGGING_FORWARD) {
      return;
    } else if(currentState == State.JOGGING_BACKWARD) {
      return;
    } else if (!beamBreak.get()) {
      currentState = State.LOADING;
    } else {
      currentState = State.IDLE;
    }
  }

  public Command jogForwardCommand() {
    return runEnd(() -> currentState = State.JOGGING_FORWARD, () -> stop());
  }

  public Command jogBackwardCommand() {
    return runEnd(() -> currentState = State.JOGGING_BACKWARD, () -> stop());
  }

  public void stop() {
    currentState = State.IDLE;
  }
}