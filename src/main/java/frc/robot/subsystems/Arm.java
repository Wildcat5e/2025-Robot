package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.controls.Follower;

public class Arm extends SubsystemBase {
  double WHEEL_CIRCUMFERENCE = 2.7 * Math.PI;
  double GEAR_RATIO = 48.0;
  double ENCODER_TICS_PER_INCH = GEAR_RATIO / WHEEL_CIRCUMFERENCE;
  public static final double TOLERANCE = 2.0;
  private final TalonFX motorOne = new TalonFX(2);
  private final TalonFX motorTwo = new TalonFX(3);
  private final Follower follower = new Follower(2, true);
  private State currentState;
  private double output;
  private double desiredAngle;

  public enum State {
    NOT_MOVING,
    MOVING_TO_INTAKE,
    MOVING_TO_OUTTAKE,
    JOGGING_TO_OUTTAKE,
    JOGGING_TO_INTAKE;
  }

  public Arm() {
    currentState = State.NOT_MOVING;
    motorTwo.setControl(follower);
  }

  @Override
  public void periodic() {
    switch (currentState) {
      case NOT_MOVING:
        output = 0.0;
        break;
      case MOVING_TO_OUTTAKE:
        output = 3.0;
        break;
      case MOVING_TO_INTAKE:
        output = -3.0;
        break;
      case JOGGING_TO_OUTTAKE:
        output = 1.0;
        break;
      case JOGGING_TO_INTAKE:
        output = -1.0;
        break;
    }

    motorOne.setVoltage(output);
    determineNextState(desiredAngle);
  }

  private void determineNextState(double desiredAngle) {
    double currentAngle = getCurrentAngle();
    this.desiredAngle = desiredAngle;

    if (desiredAngle > currentAngle + TOLERANCE) {
      currentState = State.MOVING_TO_OUTTAKE;
    } else if (desiredAngle < currentAngle - TOLERANCE) {
      currentState = State.MOVING_TO_INTAKE;
    } else {
      currentState = State.NOT_MOVING;
    }
  }

  private double getCurrentAngle() {
    return motorOne.getPosition().getValueAsDouble();
  }

  private void moveToOuttakePosition() {
    determineNextState(2.0);
  }

  public Command moveToIntakePositionCommand() {
    return runOnce(() -> moveToOuttakePosition());
  }

  private void moveToIntakePosition() {
    determineNextState(-2.0);
  }

  public Command moveToOuttakePositionCommand() {
    return runOnce(() ->  moveToIntakePosition());
  }

  private void stop() {
    desiredAngle = getCurrentAngle();
    currentState = State.NOT_MOVING;
  }

  public Command jogToIntakeCommand() {
    return runEnd(() -> currentState = State.JOGGING_TO_INTAKE, () -> stop());
  }

  public Command jogToOuttakeCommand() {
    return runEnd(() -> currentState = State.JOGGING_TO_OUTTAKE, () -> stop());
  }
}