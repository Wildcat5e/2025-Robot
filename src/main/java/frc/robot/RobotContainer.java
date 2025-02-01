package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorBangBangControl;
// import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ElevatorPID;

public class RobotContainer {
  private final XboxController controller = new XboxController(0);
  private final Trigger ButtonA = new Trigger(() -> controller.getAButton());
  private final Trigger ButtonB = new Trigger(() -> controller.getBButton());
  private final Trigger ButtonX = new Trigger(() -> controller.getXButton());
  private final Trigger ButtonY = new Trigger(() -> controller.getYButton());
  private final Trigger leftBumper = new Trigger(() -> controller.getLeftBumperButton());
  private final Trigger rightBumper = new Trigger(() -> controller.getRightBumperButton());
  private final Trigger backButton = new Trigger(() -> controller.getBackButton());
  private final Trigger startButton = new Trigger(() -> controller.getStartButton());
  private final Elevator elevator = new ElevatorPID();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    ButtonY.onTrue(elevator.moveToLevelOneCommand());
    ButtonB.onTrue(elevator.moveToLevelTwoCommand());
    // ButtonA.onTrue(elevator.moveToLevelThreeCommand());
    ButtonX.onTrue(elevator.moveToLevelFourCommand());
    leftBumper.whileTrue(elevator.jogUpCommand());
    rightBumper.whileTrue(elevator.jogDownCommand());
    backButton.onTrue(elevator.setCurrentPositionAsHomeCommand());
    startButton.onTrue(elevator.moveToPositionZeroCommand());
    ButtonA.onTrue(elevator.updateConfigCommand());
  }

  public Command getAutonomousCommand() {
    return null;
  }
}