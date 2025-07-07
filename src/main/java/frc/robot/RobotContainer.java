package frc.robot;

import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorBangBang;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.Extractor;
import frc.robot.subsystems.Outtake;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.cameraserver.CameraServer;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import static edu.wpi.first.units.Units.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // private Telemetry logger = new Telemetry(MaxSpeed);

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

    private final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    
    // private final Elevator elevator = new ElevatorBangBang();
    private final Outtake outtake = new Outtake();
    private final Extractor extractor = new Extractor();
    
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
      // NamedCommands.registerCommand("moveToPositionZero", elevator.moveToPositionZero());
      // NamedCommands.registerCommand("moveToLevelTwo", elevator.moveToLevelTwo());
      // NamedCommands.registerCommand("moveToLevelThree", elevator.moveToLevelThree());
      
      configureBindings();

      // DataLogManager.start();
      // DriverStation.startDataLog(DataLogManager.getLog());

      CameraServer.startAutomaticCapture();

      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private void configureBindings() {
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() ->
        drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
          .withVelocityY(-driver.getLeftX() * MaxSpeed)
          .withRotationalRate(-driver.getRightX() * MaxAngularRate)
      )
    );
    
      button4.onTrue(outtake.shoot());
      // button5.onTrue(elevator.moveToLevelThree());
      // button7.whileTrue(elevator.manualUp());
      // button8.onTrue(elevator.moveToLevelTwo());
      button9.whileTrue(extractor.manualUpCommand());
      // button10.whileTrue(elevator.manualDown());
      // button11.onTrue(elevator.moveToPositionZero());
      button12.whileTrue(extractor.manualDownCommand());
    
      driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

      // drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
      return autoChooser.getSelected();
    }
}
