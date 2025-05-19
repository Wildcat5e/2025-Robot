package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;

public class Extractor extends SubsystemBase {
  TalonFX motor = new TalonFX(17);
  State currentState;

  public enum State {
    IDLE,
    MANUAL_UP,
    MANUAL_DOWN
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
      case MANUAL_UP:
        motor.setVoltage(1.5);
        break;
      case MANUAL_DOWN:
        motor.setVoltage(-1.5);
        break;
    }
  }

  public Command manualUpCommand() {
    return runEnd(() -> currentState = State.MANUAL_UP, () -> currentState = State.IDLE);
  }

  public Command manualDownCommand() {
    return runEnd(() -> currentState = State.MANUAL_DOWN, () -> currentState = State.IDLE);
  }

}
