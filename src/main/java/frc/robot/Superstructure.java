package frc.robot;

import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Superstructure {
  private final Elevator elevator;
  
  public Superstructure(Elevator elevator) {
    this.elevator = elevator;
  }

  public SequentialCommandGroup moveElevatorToHomePositionTest() {
    return new SequentialCommandGroup(elevator.moveToHomePositionCommand(), new WaitUntilCommand(() -> elevator.isElevatorNotMoving()));
  }

  public SequentialCommandGroup moveElevatorToLevelTwoTest() {
    return new SequentialCommandGroup(elevator.moveToLevelTwoCommand(), new WaitUntilCommand(() -> elevator.isElevatorNotMoving()));
  }

  public SequentialCommandGroup moveElevatorToLevelThreeTest() {
    return new SequentialCommandGroup(elevator.moveToLevelThreeCommand(), new WaitUntilCommand(() -> elevator.isElevatorNotMoving()));
  }
}