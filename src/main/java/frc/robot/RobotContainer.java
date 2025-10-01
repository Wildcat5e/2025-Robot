// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorBangBang;
import frc.robot.subsystems.Extractor;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Outtake;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

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
    public final Limelight limelight = new Limelight(drivetrain);
    public final Elevator elevator = new ElevatorBangBang();
    public final Outtake outtake = new Outtake();
    public final Extractor extractor = new Extractor();
    public final AutoAlignCommands autoAlignCommands = new AutoAlignCommands(drivetrain, extractor, limelight);
    public final ShootingCommands shootingCommands = new ShootingCommands(elevator, outtake, extractor, autoAlignCommands);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("moveToPositionZero", elevator.moveToPositionZero());
        NamedCommands.registerCommand("moveToLevelTwo", elevator.moveToLevelTwo());
        NamedCommands.registerCommand("moveToLevelThree", elevator.moveToLevelThree());
        NamedCommands.registerCommand("waitForLoading", outtake.waitForLoading());
        NamedCommands.registerCommand("shoot", outtake.shoot());

        configureBindings();

        // DataLogManager.start();
        // DriverStation.startDataLog(DataLogManager.getLog());

        CameraServer.startAutomaticCapture();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )


        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // joystick.b().onTrue(limelight.printDistances());
        // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(limelight.leftAutoAlign());
        // joystick.rightBumper().onTrue(limelight.rightAutoAlign());

        joystick.leftTrigger().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        joystick.leftBumper().onTrue(autoAlignCommands.leftAutoAlign());
        // joystick.rightBumper().onTrue(autoAlignCommands.rightAutoAlign());
        joystick.rightTrigger().onTrue(shootingCommands.safetyStopExtractorPID());
        joystick.y().onTrue(shootingCommands.algaeAlignOver());
        joystick.b().onTrue(shootingCommands.algaeExtractOver());
        joystick.x().onTrue(shootingCommands.algaeAlignUnder());
        joystick.a().onTrue(shootingCommands.algaeExtractUnder());
        // joystick.y().onTrue(autoAlignCommands.printPose());
        // Experimental
        // button3.onTrue(limelight.testAlign());
        
    

        // Methods that return commands are ran once at init, saving the command
        // Commands.defer() is required the method to ensure the code is ran dynamically

        button1.onTrue(autoAlignCommands.alignArmToAlgae());
        button2.onTrue(extractor.moveArmOverAlgae());
        button3.onTrue(autoAlignCommands.driveToAlgae());
        button4.onTrue(extractor.removeAlgaeDown());
        button5.onTrue(autoAlignCommands.alignArmToAlgae());


        button6.onTrue(autoAlignCommands.alignArmToAlgae());
        button7.onTrue(extractor.moveArmUnderAlgae());
        button8.onTrue(autoAlignCommands.driveToAlgae());
        button9.onTrue(extractor.removeAlgaeUp());
        button10.onTrue(autoAlignCommands.alignArmToAlgae());

        // button1.whileTrue(shootingCommands.safetyStopExtractorPID());
        // button4.onTrue(outtake.shoot());
        // button5.onTrue(elevator.moveToLevelThree());
        // button8.onTrue(elevator.moveToLevelTwo());
        // button11.onTrue(elevator.moveToPositionZero());
        // button7.whileTrue(elevator.manualUp());
        // button10.whileTrue(elevator.manualDown());
        // button9.whileTrue(extractor.manualUpCommand());
        // button12.whileTrue(extractor.manualDownCommand());

        // drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
