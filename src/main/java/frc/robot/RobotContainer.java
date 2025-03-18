package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.generated.TunerConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import static edu.wpi.first.units.Units.*;
import org.opencv.core.Core;
import org.opencv.core.Mat;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final CommandXboxController driver = new CommandXboxController(0);
    // private final CommandXboxController operator = new CommandXboxController(1);
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

    Thread m_visionThread;

    public RobotContainer() {
        configureBindings();
        
        NamedCommands.registerCommand("moveToHomePositionCommand", superstructure.moveElevatorToHomePositionTest());
        NamedCommands.registerCommand("moveToLevelTwoCommand", superstructure.moveElevatorToLevelTwoTest());
        NamedCommands.registerCommand("moveToLevelThreeCommand", superstructure.moveElevatorToLevelThreeTest());
        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        CameraServer.startAutomaticCapture();
        m_visionThread =
        new Thread(
            () -> {
              // Get the UsbCamera from CameraServer
              UsbCamera camera = CameraServer.startAutomaticCapture();
              // Set the resolution
              camera.setResolution(640,480);
              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();
              // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

              // Mats are very memory expensive. Lets reuse this Mat.
              Mat mat = new Mat();

              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              Mat rotatedImage = new Mat();
              while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }
                Core.rotate(mat, rotatedImage, Core.ROTATE_180);
                // Give the output stream a new image to display
                outputStream.putFrame(rotatedImage);
              }
            });
    m_visionThread.setDaemon(true);
    // m_visionThread.start();
    }

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getRightY() * MaxSpeed)
                    .withVelocityY(-driver.getRightX() * MaxSpeed)
                    .withRotationalRate(-driver.getLeftX() * MaxAngularRate)
            )
        );
        
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // drivetrain.registerTelemetry(logger::telemeterize);
        button5.onTrue(superstructure.moveElevatorToLevelThreeTest());
        button7.whileTrue(elevator.jogUpCommand());
        button8.onTrue(superstructure.moveElevatorToLevelTwoTest());
        button9.whileTrue(outtake.jogBackwardCommand());
        button10.whileTrue(elevator.jogDownCommand());
        button11.onTrue(superstructure.moveElevatorToHomePositionTest());
        button12.whileTrue(outtake.jogForwardCommand());

        // operator.x().whileTrue(elevator.jogDownCommand());
        // operator.y().whileTrue(elevator.jogUpCommand());
        // operator.rightTrigger().whileTrue(outtake.jogForwardCommand());
        // operator.leftTrigger().whileTrue(outtake.jogBackwardCommand());

        // operator.povUp().onTrue(superstructure.moveElevatorToHomePositionTest());
        // operator.povRight().onTrue(superstructure.moveElevatorToLevelTwoTest());
        // operator.povDown().onTrue(superstructure.moveElevatorToLevelThreeTest());
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(new PathPlannerAuto("Mid 1 Coral Auto"), new ParallelDeadlineGroup(new WaitCommand(2.5), outtake.jogForwardCommand()));
    }
}
