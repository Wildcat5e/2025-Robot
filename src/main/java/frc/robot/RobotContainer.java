package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MoveElevatorToPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ClimbingHooks;

public class RobotContainer {
  XboxController controller = new XboxController(0);
  Trigger ButtonA = new Trigger(() -> controller.getAButton());
  Trigger ButtonB = new Trigger(() -> controller.getBButton());
  Trigger ButtonX = new Trigger(() -> controller.getXButton());
  Trigger ButtonY = new Trigger(() -> controller.getYButton());
  Elevator elevator = new Elevator();
  ClimbingHooks climbingHooks = new ClimbingHooks();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    ButtonA.onTrue(new MoveElevatorToPosition(elevator, (Double) null));

    ButtonB.onTrue(climbingHooks.extendSolenoid(true));

    ButtonX.onTrue(climbingHooks.extendSolenoid(false));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}