package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extractor extends SubsystemBase {
  TalonFX motor = new TalonFX(17);
  State currentState;

  public enum State {
    IDLE,
    JOG_UP,
    JOG_DOWN
  }

  public Extractor() {
    currentState = State.IDLE;
  }

  @Override
  public void periodic() {
    switch (currentState) {
      case IDLE:
        motor.setVoltage(0);
        break;
      case JOG_UP:
        motor.setVoltage(3);
        break;
      case JOG_DOWN:
        motor.setVoltage(-3);
        break;
    }
  }

  public Command jogUp() {
    return runEnd(() -> currentState = State.JOG_UP, () -> currentState = State.IDLE);
  }

  public Command jogDown() {
    return runEnd(() -> currentState = State.JOG_DOWN, () -> currentState = State.IDLE);
  }

}
