package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveToPosition extends Command {
  private final Elevator elevator;
  private final double desiredHeight;

  public MoveToPosition(Elevator elevator, double desiredHeight) {
    this.elevator = elevator;
    this.desiredHeight = desiredHeight;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.move(desiredHeight);
  }

  @Override
  public boolean isFinished() {
    return elevator.atDesiredHeight();
  }
}
