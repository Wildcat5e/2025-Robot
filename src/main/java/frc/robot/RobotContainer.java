package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.MoveToPosition;

public class RobotContainer {
  private static final int LEVEL_ONE = 1;
  private static final int LEVEL_TWO = 2;
  Elevator elevator = new Elevator();
  XboxController controller = new XboxController(0);
  Trigger ButtonA = new Trigger(() -> controller.getAButton());
  Trigger ButtonB = new Trigger(() -> controller.getBButton());
  Trigger ButtonX = new Trigger(() -> controller.getXButton());
  Trigger ButtonY = new Trigger(() -> controller.getYButton());

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    ButtonA.onTrue(new MoveToPosition(elevator, LEVEL_ONE));
    ButtonB.onTrue(new MoveToPosition(elevator, LEVEL_TWO));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
