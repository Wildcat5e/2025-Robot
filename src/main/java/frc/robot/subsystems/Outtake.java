package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LED;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class Outtake extends SubsystemBase {
  private final TalonFX leftMotor = new TalonFX(15);
  private final TalonFX rightMotor = new TalonFX(16);
  private final DigitalInput startBeamBreak = new DigitalInput(1);
  private final LED led;
  private State currentState;
  private BooleanPublisher startBeamBreakPublisher;
  private StringPublisher currentStatePublisher;
  private double leftVolts;
  private double rightVolts;
  private long shootingStartTime = 0;
  private long shootingEndTime;
  private long shootingDeltaTime;
  private State lastState = null;

  public enum State {
    IDLE,
    LOADING,
    SHOOTING
  }

  public Outtake(LED led) {
    currentState = State.IDLE;
    lastState = null;

    NetworkTable outtake = NetworkTableInstance.getDefault().getTable("Outtake");
    startBeamBreakPublisher = outtake.getBooleanTopic("Start Beam Break").publish();
    currentStatePublisher = outtake.getStringTopic("Current State").publish();
    SmartDashboard.putData(this);

    this.led = led;
  }

  @Override
  public void periodic() {
    lastState = currentState;

    handleStateTransition();

    if(currentState != lastState) {
      if(currentState == State.IDLE) {
        led.setLEDs(LED.BLACK, LED.BLOCK_2[0], LED.BLOCK_2[1]);
      } else if(currentState == State.LOADING) {
        led.setLEDs(LED.BLUE, LED.BLOCK_2[0], LED.BLOCK_2[1]);
      } else if(currentState == State.SHOOTING) {
        led.setLEDs(LED.ORANGE, LED.BLOCK_2[0], LED.BLOCK_2[1]);
      }
    }

    startBeamBreakPublisher.set(startBeamBreak.get());
    currentStatePublisher.set(currentState.toString());

    switch (currentState) {
      case IDLE:
        leftVolts = 0.0;
        rightVolts = 0.0;
        break;
      case LOADING:
        leftVolts = -12.0;
        rightVolts = 8.0;
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
    if (!startBeamBreak.get()) {
      currentState = State.LOADING;
    } else if (currentState == State.SHOOTING && shootingStartTime == 0) {
      shootingStartTime = System.currentTimeMillis();
    } else if (shootingStartTime > 0) {
      shootingEndTime = System.currentTimeMillis();
      shootingDeltaTime = shootingEndTime - shootingStartTime;
      if (shootingDeltaTime >= 500) {
        currentState = State.IDLE;
        shootingStartTime = 0;
      }
    } else if (startBeamBreak.get()) {
      currentState = State.IDLE;
    }
  }

  public Command shoot() {
    return runOnce(() -> currentState = State.SHOOTING).withName("Shoot");
  }
}