package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorBangBangControl;
import frc.robot.subsystems.ElevatorFalconPID;
import frc.robot.subsystems.ElevatorRioPID;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Outtake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
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
  private final Elevator elevator = new ElevatorFalconPID();
  private final Outtake outtake = new Outtake();
  private final Limelight limelight = new Limelight();
  private final Superstructure superstructure = new Superstructure(elevator, outtake, limelight);
  private Command m_autonomousCommand;

  public Robot() {
    configureBindings();
  }

  private void configureBindings() {
    button1.onTrue(limelight.alignLeft());
    button3.onTrue(limelight.alignRight());
    button11.onTrue(superstructure.moveElevatorToLevelOneAndOuttake());
    button8.onTrue(superstructure.moveElevatorToLevelTwoAndOuttake());
    button5.onTrue(superstructure.moveElevatorToLevelThreeAndOuttake());
    button4.whileTrue(elevator.jogUpCommand());
    button6.whileTrue(elevator.jogDownCommand());
    button7.whileTrue(outtake.jogForwardCommand());
    button9.whileTrue(outtake.jogBackwardCommand());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}