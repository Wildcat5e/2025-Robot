package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralArm extends SubsystemBase {
  public CoralArm() {}

  @Override
  public void periodic() {
    
  }
  
  public void moveToIntakePosition() {
    System.out.println("Moving to intake position");
  }

  public Command moveToIntakePositionCommand() {
    return runOnce(() -> moveToIntakePosition());
  }

  public void moveToOuttakePosition() {
    System.out.println("Moving to outtake position");
  }

  public Command moveToOuttakePositionCommand() {
    return runOnce(() -> moveToOuttakePosition());
  }
}