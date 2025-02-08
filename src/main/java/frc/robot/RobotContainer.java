package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.ElevatorBangBangControl;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final XboxController controller = new XboxController(0);
  private final Joystick joystick = new Joystick(1);
  private final JoystickButton button1 = new JoystickButton(joystick, 1);
  private final JoystickButton button2 = new JoystickButton(joystick, 2);
  private final JoystickButton button3 = new JoystickButton(joystick, 3);
  private final JoystickButton button4 = new JoystickButton(joystick, 4);
  private final JoystickButton button5 = new JoystickButton(joystick, 5);
  private final JoystickButton button6 = new JoystickButton(joystick, 6);
  private final JoystickButton button7 = new JoystickButton(joystick, 7);
  private final JoystickButton button8 = new JoystickButton(joystick, 8);
  private final JoystickButton button9 = new JoystickButton(joystick, 9);
  private final JoystickButton button10 = new JoystickButton(joystick, 10);
  private final JoystickButton button11 = new JoystickButton(joystick, 11);
  private final JoystickButton button12 = new JoystickButton(joystick, 12);
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
    button9.onTrue(arm.moveToIntakePositionCommand());
    button10.onTrue(arm.moveToOuttakePositionCommand());
    button11.onTrue(arm.jogToIntakeCommand());
    button12.onTrue(arm.jogToOuttakeCommand());
  }

  public Command getAutonomousCommand() {
    return null;
  }
}