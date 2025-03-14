package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;

public class Outtake extends SubsystemBase {
  public static final double TOLERANCE = 0.0;
  private final TalonFX motor = new TalonFX(15);
  private final DigitalInput beamBreak = new DigitalInput(1);
  private State currentState;
  private double output;
    private BooleanSubscriber outtakeSubscriber;
  private BooleanPublisher outtakePublisher;

  public enum State {
    NOT_MOVING,
    MOVING_FORWARD,
    MOVING_BACKWARD,
    JOGGING_FORWARD,
    JOGGING_BACKWARD;
  }

  public Outtake() {
    currentState = State.NOT_MOVING;
    NetworkTable outtake = NetworkTableInstance.getDefault().getTable("Outtake");
    outtakeSubscriber = outtake.getBooleanTopic("outtakeSubscriber").subscribe(false);
    outtakePublisher = outtake.getBooleanTopic("outtakePublisher").publish();
  }

  @Override
  public void periodic() {
    outtakePublisher.set(beamBreak.get());
    if(!beamBreak.get()) {
      if(motor.getPosition().getValueAsDouble() < 5.0) {
        motor.setVoltage(1.0);
      }
    }

    switch (currentState) {
      case NOT_MOVING:
        output = 0.0;
        break;
      case MOVING_FORWARD:
        output = 2.0;
        break;
      case MOVING_BACKWARD:
        output = -2.0;
        break;
      case JOGGING_FORWARD:
        output = 1.0;
        break;
      case JOGGING_BACKWARD:
        output = -1.0;
        break;
    }

    motor.setVoltage(output);
  }

  public Command jogForwardCommand() {
    return runEnd(() -> currentState = State.JOGGING_FORWARD, () -> stop());
  }

  public Command jogBackwardCommand() {
    return runEnd(() -> currentState = State.JOGGING_BACKWARD, () -> stop());
  }

  public void stop() {
    currentState = State.NOT_MOVING;
  }
}