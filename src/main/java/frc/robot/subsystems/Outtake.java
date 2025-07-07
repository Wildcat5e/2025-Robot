package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {
  private final TalonFX leftMotor = new TalonFX(15);
  private final TalonFX rightMotor = new TalonFX(16);
  private final DigitalInput entryBeamBreak = new DigitalInput(1);
  private State currentState;
  private double leftVolts;
  private double rightVolts;
  private long shootingStartTime = 0;
  private long shootingEndTime;
  private long shootingDeltaTime;
  private BooleanPublisher startBeamBreakPublisher;
  private StringPublisher currentStatePublisher;

  public enum State {
    WAITING,
    LOADING,
    HOLDING,
    SHOOTING
  }

  public Outtake() {
    currentState = State.WAITING;

    NetworkTable outtake = NetworkTableInstance.getDefault().getTable("Outtake");
    startBeamBreakPublisher = outtake.getBooleanTopic("Start Beam Break").publish();
    currentStatePublisher = outtake.getStringTopic("Current State").publish();
    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    handleStateTransition();

    startBeamBreakPublisher.set(entryBeamBreak.get());
    currentStatePublisher.set(currentState.toString());

    switch (currentState) {
      case WAITING:
        leftVolts = 0.0;
        rightVolts = 0.0;
        break;
      case LOADING:
        leftVolts = -12.0;
        rightVolts = 8.0;
        break;
      case HOLDING:
        leftVolts = 0.0;
        rightVolts = 0.0;
        break;
      case SHOOTING:
        leftVolts = -12.0;
        rightVolts = 8.0;
        break;
    }

    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(rightVolts);
  }

  private void handleStateTransition() {    
    if (currentState == State.WAITING && !entryBeamBreak.get()) {
      currentState = State.LOADING;
    } else if (currentState == State.LOADING && entryBeamBreak.get()) {
      currentState = State.HOLDING;
    } else if (currentState == State.SHOOTING && shootingStartTime == 0) {
      shootingStartTime = System.currentTimeMillis();
    } else if (shootingStartTime > 0) {
      shootingEndTime = System.currentTimeMillis();
      shootingDeltaTime = shootingEndTime - shootingStartTime;
      if (shootingDeltaTime >= 500) {
        currentState = State.WAITING;
        shootingStartTime = 0;
      }
    }
  }

  public Command shoot() {
    return runOnce(() -> currentState = State.SHOOTING).withName("Shoot");
  }
}
