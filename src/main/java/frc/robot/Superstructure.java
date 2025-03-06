package frc.robot;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Superstructure {
  private final Elevator elevator;
  private final Outtake outtake;
  private final Limelight limelight;
  
  public Superstructure(Elevator elevator, Outtake outtake, Limelight limelight) {
    this.elevator = elevator;
    this.outtake = outtake;
    this.limelight = limelight;
  }
  
  public SequentialCommandGroup moveElevatorToLevelOneAndOuttake() {
    return new SequentialCommandGroup(elevator.moveToLevelOneCommand(), new WaitUntilCommand(() -> elevator.isElevatorNotMoving()), new ParallelDeadlineGroup(new WaitCommand(1.5), outtake.outtakeCommand()), elevator.moveToHomePositionCommand(), new WaitUntilCommand(() -> elevator.isElevatorNotMoving()));
  }

  public SequentialCommandGroup moveElevatorToLevelTwoAndOuttake() {
    return new SequentialCommandGroup(elevator.moveToLevelTwoCommand(), new WaitUntilCommand(() -> elevator.isElevatorNotMoving()), new ParallelDeadlineGroup(new WaitCommand(1.5), outtake.outtakeCommand()), elevator.moveToHomePositionCommand(), new WaitUntilCommand(() -> elevator.isElevatorNotMoving()));
  }
  
  public SequentialCommandGroup moveElevatorToLevelThreeAndOuttake() {
    return new SequentialCommandGroup(elevator.moveToLevelThreeCommand(), new WaitUntilCommand(() -> elevator.isElevatorNotMoving()), new ParallelDeadlineGroup(new WaitCommand(1.5), outtake.outtakeCommand()), elevator.moveToHomePositionCommand(), new WaitUntilCommand(() -> elevator.isElevatorNotMoving()));
  }
}