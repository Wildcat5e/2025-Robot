package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.State;

public class MoveElevatorToPosition extends Command {
  Elevator elevator;
  double desiredHeight;

  public MoveElevatorToPosition(Elevator elevator, double desiredHeight) {
    this.elevator = elevator;
    this.desiredHeight = desiredHeight;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.applyMovementLogic(desiredHeight);
  }

  @Override
  public boolean isFinished() {
    return elevator.getCurrentState() == State.NOT_MOVING;
  }
}
