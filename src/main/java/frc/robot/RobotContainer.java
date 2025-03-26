package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.generated.TunerConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.units.Units.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final Joystick operator = new Joystick(1);
    private final JoystickButton button1 = new JoystickButton(operator, 1);
    private final JoystickButton button2 = new JoystickButton(operator, 2);
    private final JoystickButton button3 = new JoystickButton(operator, 3);
    private final JoystickButton button4 = new JoystickButton(operator, 4);
    private final JoystickButton button5 = new JoystickButton(operator, 5);
    private final JoystickButton button6 = new JoystickButton(operator, 6);
    private final JoystickButton button7 = new JoystickButton(operator, 7);
    private final JoystickButton button8 = new JoystickButton(operator, 8);
    private final JoystickButton button9 = new JoystickButton(operator, 9);
    private final JoystickButton button10 = new JoystickButton(operator, 10);
    private final JoystickButton button11 = new JoystickButton(operator, 11);
    private final JoystickButton button12 = new JoystickButton(operator, 12);

    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Elevator elevator = new Elevator();
    private final Outtake outtake = new Outtake();
    private final Superstructure superstructure = new Superstructure(elevator);
    
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
      NamedCommands.registerCommand("moveToHomePositionCommand", superstructure.moveElevatorToHomePositionTest());
      NamedCommands.registerCommand("moveToLevelTwoCommand", superstructure.moveElevatorToLevelTwoTest());
      NamedCommands.registerCommand("moveToLevelThreeCommand", superstructure.moveElevatorToLevelThreeTest());
      
      configureBindings();
      
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);

      CameraServer.startAutomaticCapture();
    }

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getRightY() * MaxSpeed)
                    .withVelocityY(-driver.getRightX() * MaxSpeed)
                    .withRotationalRate(-driver.getLeftX() * MaxAngularRate)
            )
        );

        button5.onTrue(superstructure.moveElevatorToLevelThreeTest());
        button6.onTrue(outtake.shootCommand());
        button7.whileTrue(elevator.manualUpCommand());
        button8.onTrue(superstructure.moveElevatorToLevelTwoTest());
        button9.whileTrue(outtake.manualBackwardCommand());
        button10.whileTrue(elevator.manualDownCommand());
        button11.onTrue(superstructure.moveElevatorToHomePositionTest());
        button12.whileTrue(outtake.manualForwardCommand());
    }

    public Command getAutonomousCommand() {
      return autoChooser.getSelected();
    }
}
