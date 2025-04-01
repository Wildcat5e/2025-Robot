package frc.robot;

import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Superstructure {
  private final Elevator elevator;

  public Superstructure(Elevator elevator) {
    this.elevator = elevator;
  }

  public SequentialCommandGroup moveElevatorToPositionZeroCommand() {
    return new SequentialCommandGroup(elevator.moveToPositionZeroCommand(), new WaitUntilCommand(() -> elevator.isElevatorNotMoving()));
  }

  public SequentialCommandGroup moveElevatorToLevelTwoCommand() {
    return new SequentialCommandGroup(elevator.moveToLevelTwoCommand(), new WaitUntilCommand(() -> elevator.isElevatorNotMoving()));
  }

  public SequentialCommandGroup moveElevatorToLevelThreeCommand() {
    return new SequentialCommandGroup(elevator.moveToLevelThreeCommand(), new WaitUntilCommand(() -> elevator.isElevatorNotMoving()));
  }
}