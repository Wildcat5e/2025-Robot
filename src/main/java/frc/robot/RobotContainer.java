package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.ElevatorBangBangControl;
import frc.robot.subsystems.ElevatorPID;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.ctre.phoenix6.SignalLogger;

public class RobotContainer {
  private final XboxController controller = new XboxController(0);
  private final Trigger aButton = new Trigger(() -> controller.getAButton());
  private final Trigger bButton = new Trigger(() -> controller.getBButton());
  private final Trigger xButton = new Trigger(() -> controller.getXButton());
  private final Trigger yButton = new Trigger(() -> controller.getYButton());
  private final Trigger leftBumper = new Trigger(() -> controller.getLeftBumperButton());
  private final Trigger rightBumper = new Trigger(() -> controller.getRightBumperButton());
  private final Trigger backButton = new Trigger(() -> controller.getBackButton());
  private final Trigger startButton = new Trigger(() -> controller.getStartButton());
  private final Elevator elevator = new ElevatorPID();

  public RobotContainer() {
    configureBindings();
    SignalLogger.setPath("/home/lvuser/logs");
  }

  private void configureBindings() {
    aButton.onTrue(elevator.moveToLevelOneCommand());
    bButton.onTrue(elevator.moveToLevelTwoCommand());
    xButton.onTrue(elevator.moveToLevelThreeCommand());
    yButton.onTrue(elevator.moveToLevelFourCommand());
    leftBumper.whileTrue(elevator.jogUpCommand());
    rightBumper.whileTrue(elevator.jogDownCommand());
    backButton.onTrue(elevator.setCurrentPositionAsHomeCommand());
  }

  public Command getAutonomousCommand() {
    return null;
  }
}