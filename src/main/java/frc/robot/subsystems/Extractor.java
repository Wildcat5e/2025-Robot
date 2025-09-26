package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class Extractor extends SubsystemBase {
  // meters all units
  private static final double TOLERANCE = 0.05;
  private static final double GEAR_RATIO = 36;
  private static final double ARM_LENGTH = .572;
  private static final double ARM_CIRCUMFERENCE = 2 * Math.PI * ARM_LENGTH;
  // under position meaning arm will be below algae and push up
  // over position meaning arm will be above algae and drag out
  private static final double UNDER_ALGAE = 1;
  private static final double OVER_ALGAE = 1.4;


  TalonFX motor = new TalonFX(17);
  State currentState;
  double currentHeight;
  double targetHeight;

  double counter = 0;

  public enum State {
    IDLE,
    UP,
    DOWN,
    SLOW_DOWN,
    SLOW_UP,
    MANUAL_UP,
    MANUAL_DOWN
  }

  public Extractor() {
    currentState = State.IDLE;
    motor.setPosition(0);
    targetHeight = 0;
  }

  @Override
  public void periodic() {

    currentHeight = getCurrentHeight();

    handleStateTransition(currentHeight);

    switch (currentState) {
      case IDLE:
        motor.setVoltage(0);
        break;
      case UP:
        motor.setVoltage(1.5);
        break;
      case DOWN:
        motor.setVoltage(-1.5);
        break;
      case MANUAL_UP:
        motor.setVoltage(1.5);
        break;
      case MANUAL_DOWN:
        motor.setVoltage(-1.5);
        break;
      case SLOW_DOWN:
        motor.setVoltage(-0.5);
        break;
      case SLOW_UP:
        motor.setVoltage(0.5);
        break;
    }

    counter++;
    if (counter % 100 == 0){
      System.out.println("current height: " + currentHeight);
      System.out.println("within tolerance?: " + withinTolerance());
    }

  }

  public boolean withinTolerance(){
    return Math.abs(currentHeight - targetHeight) <= TOLERANCE;
  }

  public void handleStateTransition(double currentHeight){
    if (currentState == State.MANUAL_DOWN || currentState == State.MANUAL_UP){
      return;
    } else if (targetHeight >= currentHeight + TOLERANCE){
      currentState = State.UP;
    } else if (targetHeight <= currentHeight - TOLERANCE){
      currentState = State.DOWN;
    } else {
      currentState = State.IDLE;
    }
  }

  public void setTargetHeight(double targetHeight){
    this.targetHeight = targetHeight;
  }

  public double getCurrentHeight(){
    return motor.getPosition().getValueAsDouble() / GEAR_RATIO * ARM_CIRCUMFERENCE;
  }

  public Command setHeightZero(){
    return runOnce(() -> {
      motor.setPosition(0);
      setTargetHeight(0);});
  }

  public Command moveArmUnderAlgae(){
    return new FunctionalCommand(
      () -> {
        setTargetHeight(UNDER_ALGAE);
      System.out.println("INIT");}
      , 
      () -> {
        System.out.println("current height: " + currentHeight);
        System.out.println("target height: " + targetHeight);
      }, 
      (interrupted) -> {}, 
      () -> withinTolerance(), 
      this
      );
  }

  public Command moveArmOverAlgae(){
    return new FunctionalCommand(
      () -> setTargetHeight(OVER_ALGAE), 
      () -> {}, 
      (interrupted) -> {}, 
      () -> withinTolerance(), 
      this
      );
  }

  public Command manualUpCommand() {
    return runEnd(() -> currentState = State.MANUAL_UP, 
    () -> stop());
  }

  public Command manualDownCommand() {
    return runEnd(() -> currentState = State.MANUAL_DOWN, () -> stop());
  }

  private void stop() {
    currentState = State.IDLE;
    setTargetHeight(getCurrentHeight());
}

}
