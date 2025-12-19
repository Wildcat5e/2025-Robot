package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Photon;

import static edu.wpi.first.units.Units.*;

public class Robot extends TimedRobot {

    private static final SwerveRequest.FieldCentric SWERVE_REQUEST = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
            .withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Photon photon = new Photon(drivetrain);

    private final Command autonomous;

    public Robot() {
        SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        autonomous = autoChooser.getSelected();

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() ->
                        SWERVE_REQUEST.withVelocityX(-joystick.getLeftY() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
                                .withVelocityY(-joystick.getLeftX() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
                                .withRotationalRate(-joystick.getRightX() * RotationsPerSecond.of(0.75).in(RadiansPerSecond))
                )
        );

        RobotModeTriggers
                .disabled()
                .whileTrue(drivetrain.applyRequest(SwerveRequest.Idle::new).ignoringDisable(true));

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        joystick.leftTrigger().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }

    @Override public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override public void autonomousInit() {
        if (autonomous != null) autonomous.schedule();
    }

    @Override public void teleopInit() {
        if (autonomous != null) autonomous.cancel();
    }

    @Override public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}
