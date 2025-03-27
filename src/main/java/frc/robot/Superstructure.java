package frc.robot;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Superstructure {
  private final Elevator elevator;
  private final Outtake outtake;
  private final Vision vision;
  private final Drivetrain drivetrain;

  public Superstructure(Elevator elevator, Outtake outtake, Vision vision, Drivetrain drivetrain) {
    this.elevator = elevator;
    this.outtake = outtake;
    this.vision = vision;
    this.drivetrain = drivetrain;
  }

  public SequentialCommandGroup moveElevatorToLevelTwoCommands() {
    return new SequentialCommandGroup(
        elevator.moveToLevelTwoCommand(), new WaitUntilCommand(() -> elevator.isElevatorNotMoving())
      );
  }

  public SequentialCommandGroup moveElevatorToLevelThreeCommand() {
    return new SequentialCommandGroup(
        elevator.moveToLevelThreeCommand(), new WaitUntilCommand(() -> elevator.isElevatorNotMoving())
      );
  }

  public SequentialCommandGroup shootOuttakeAndMoveElevatorToHomePosition() {
    return new SequentialCommandGroup(
        outtake.shootCommand(), new WaitUntilCommand(() -> outtake.isOuttakeNotShooting()),
        elevator.moveToHomePositionCommand(), new WaitUntilCommand(() -> elevator.isElevatorNotMoving()));
  }
}