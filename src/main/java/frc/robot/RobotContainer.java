// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorBangBangControl;
import frc.robot.subsystems.Outtake;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller = new CommandXboxController(0);
    private final Joystick numberpad = new Joystick(1);
    private final JoystickButton buttonEleven = new JoystickButton(numberpad, 11);
    private final JoystickButton buttonEight = new JoystickButton(numberpad, 8);
    private final JoystickButton buttonFive = new JoystickButton(numberpad, 5);
    private final JoystickButton buttonTen = new JoystickButton(numberpad, 10);
    private final JoystickButton buttonSeven = new JoystickButton(numberpad, 7);
    private final JoystickButton buttonNine = new JoystickButton(numberpad, 9);
    private final JoystickButton buttonTwelve = new JoystickButton(numberpad, 12);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser;

    private final Elevator elevator = new ElevatorBangBangControl();
    // private final Outtake outtake = new Outtake();
    private final Superstructure superstructure = new Superstructure(elevator);

    public RobotContainer() {
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        NamedCommands.registerCommand("moveToHomePositionCommand", superstructure.moveElevatorToHomePositionTest());
        NamedCommands.registerCommand("moveToLevelTwoCommand", superstructure.moveElevatorToLevelTwoTest());
        NamedCommands.registerCommand("moveToLevelThreeCommand", superstructure.moveElevatorToLevelThreeTest());
        // NamedCommands.registerCommand("outtakeCommand", outtake.outtakeCommand());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-controller.getRightY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller.getRightX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(controller.getLeftX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        buttonEleven.onTrue(superstructure.moveElevatorToHomePositionTest());
        buttonEight.onTrue(superstructure.moveElevatorToLevelTwoTest());
        buttonFive.onTrue(superstructure.moveElevatorToLevelThreeTest());
        buttonNine.whileTrue(elevator.jogDownCommand());
        buttonSeven.whileTrue(elevator.jogUpCommand());
        // buttonTen.whileTrue(outtake.jogBackwardCommand());
        // buttonTwelve.whileTrue(outtake.jogForwardCommand());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
