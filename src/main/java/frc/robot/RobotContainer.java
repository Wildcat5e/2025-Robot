package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.ElevatorBangBangControl;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final XboxController controller = new XboxController(0);
  private final Joystick joystick = new Joystick(1);
  private final Trigger aButton = new Trigger(() -> controller.getAButton());
  private final Trigger bButton = new Trigger(() -> controller.getBButton());
  private final Trigger xButton = new Trigger(() -> controller.getXButton());
  private final Trigger yButton = new Trigger(() -> controller.getYButton());
  private final Trigger leftBumper = new Trigger(() -> controller.getLeftBumperButton());
  private final Trigger rightBumper = new Trigger(() -> controller.getRightBumperButton());
  private final Trigger backButton = new Trigger(() -> controller.getBackButton());
  private final JoystickButton button1 = new JoystickButton(joystick, 1);
  private final JoystickButton button2 = new JoystickButton(joystick, 2);
  private final JoystickButton button3 = new JoystickButton(joystick, 3);
  private final JoystickButton button4 = new JoystickButton(joystick, 4);
  private final JoystickButton button5 = new JoystickButton(joystick, 5);
  private final JoystickButton button6 = new JoystickButton(joystick, 6);
  private final JoystickButton button7 = new JoystickButton(joystick, 7);
  private final JoystickButton button8 = new JoystickButton(joystick, 8);
  private final Elevator elevator = new ElevatorPID();
  private final Arm arm = new Arm();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    button1.whileTrue(elevator.jogUpCommand());
    button2.whileTrue(elevator.jogDownCommand());
    button3.onTrue(elevator.moveToCoralStationHeightCommand());
    button4.onTrue(elevator.moveToPositionZeroCommand());
    button5.onTrue(elevator.moveToLevelOneCommand());
    button6.onTrue(elevator.moveToLevelTwoCommand());
    button7.onTrue(elevator.moveToLevelThreeCommand());
    button8.onTrue(elevator.moveToLevelFourCommand());
  }

  public Command getAutonomousCommand() {
    return null;
  }
}