// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PaulElevator;
import frc.robot.subsystems.PaulElevator.State;

/**
 * Example ONLY:  Command to use Elevator as a State Machine.
 */
public class PaulMoveElevator extends Command {
  private PaulElevator elevator;
    private double desiredHeight;
    
      /** Creates a new PaulMoveElevator. */
      public PaulMoveElevator(PaulElevator elevator, double desiredHeight) {
        this.elevator = elevator;
      this.desiredHeight = desiredHeight;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.moveTo(desiredHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted){
      elevator.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.getState() == State.NOT_MOVING;
  }
}
