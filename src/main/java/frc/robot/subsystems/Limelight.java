package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private static final double POSITION_TOLERANCE = 0.05;
  private static final double ROTATION_TOLERANCE = 0.05;

  private static final Transform2d LEFT_ALIGN_DISTANCE = new Transform2d(
      new Translation2d(0.5, -0.3),
      Rotation2d.fromDegrees(0));

  private static final Transform2d RIGHT_ALIGN_DISTANCE = new Transform2d(
      new Translation2d(0.5, 0.3),
      Rotation2d.fromDegrees(0));

  double minDistance = 0.3;

  private static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  private static final List<Pose2d> blueAprilTagPoses = new ArrayList<Pose2d>();
  private static final List<Pose2d> redAprilTagPoses = new ArrayList<Pose2d>();

  private static final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

  Drivetrain drivetrain;
  Field2d field = new Field2d();
  NetworkTableEntry botPoseEntry;
  double[] botPose;
  List<Pose2d> aprilTagPoses;
  double[] stddevsDoubleArray;
  boolean hasTarget;
  Pose2d updatedPose;
  double totalLatencyMs;
  double timestamp;
  double tagCount;
  Matrix<N3, N1> stddevs;
  NetworkTableEntry stddevsEntry;
  Pose2d targetPose;

  public Limelight(Drivetrain drivetrain) {

    blueAprilTagPoses.add(layout.getTagPose(17).get().toPose2d());
    blueAprilTagPoses.add(layout.getTagPose(18).get().toPose2d());
    blueAprilTagPoses.add(layout.getTagPose(19).get().toPose2d());
    blueAprilTagPoses.add(layout.getTagPose(20).get().toPose2d());
    blueAprilTagPoses.add(layout.getTagPose(21).get().toPose2d());
    blueAprilTagPoses.add(layout.getTagPose(22).get().toPose2d());

    redAprilTagPoses.add(layout.getTagPose(17).get().toPose2d());
    redAprilTagPoses.add(layout.getTagPose(18).get().toPose2d());
    redAprilTagPoses.add(layout.getTagPose(19).get().toPose2d());
    redAprilTagPoses.add(layout.getTagPose(20).get().toPose2d());
    redAprilTagPoses.add(layout.getTagPose(21).get().toPose2d());
    redAprilTagPoses.add(layout.getTagPose(22).get().toPose2d());

    this.drivetrain = drivetrain;

    stddevsEntry = limelight.getEntry("stddevs");

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Blue) {
      botPoseEntry = limelight.getEntry("botpose_wpiblue");
      aprilTagPoses = blueAprilTagPoses;
    } else {
      aprilTagPoses = redAprilTagPoses;
    }

    SmartDashboard.putData("Field", field);
    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    hasTarget = limelight.getEntry("tv").getDouble(0.0) == 1.0;

    if (hasTarget) {
      botPose = botPoseEntry.getDoubleArray(new double[11]);
      updatedPose = new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));
      totalLatencyMs = botPose[6];
      timestamp = Timer.getFPGATimestamp() - totalLatencyMs / 1000;
      tagCount = (double) botPose[7];
      stddevsDoubleArray = stddevsEntry.getDoubleArray(new double[11]);

      if (tagCount == 1.0) {
        stddevs = VecBuilder.fill(stddevsDoubleArray[0], stddevsDoubleArray[1], stddevsDoubleArray[5]);
      } else if (tagCount > 1.0) {
        stddevs = VecBuilder.fill(stddevsDoubleArray[6], stddevsDoubleArray[7], stddevsDoubleArray[11]);
      }

      drivetrain.addVisionMeasurement(updatedPose, timestamp, stddevs);
    }

    field.setRobotPose(drivetrain.getState().Pose);
  }

  public Command rightAutoAlign() {
    return Commands.defer(() -> {
      Pose2d currentPose = drivetrain.getState().Pose;
      ChassisSpeeds speeds = drivetrain.getState().Speeds;

      Rotation2d directionOfTravel;
      double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

      // CHECK TO SEE IF PATH WILL RUN IF TARGET POSE IS NULL
      Pose2d targetPose = null;
      Pose2d closestTag = currentPose.nearest(aprilTagPoses);
      double distanceToClosestTag = currentPose.getTranslation().getDistance(closestTag.getTranslation());

      if (distanceToClosestTag <= minDistance) {
        targetPose = closestTag.transformBy(RIGHT_ALIGN_DISTANCE);
        this.targetPose = targetPose;
      }

      if (speed < 0.25) {
        Translation2d diff = targetPose.getTranslation().minus(currentPose.getTranslation());
        directionOfTravel = diff.getAngle();
      } else {
        directionOfTravel = new Rotation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      }

      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
          new Pose2d(
              currentPose.getX(),
              currentPose.getY(),
              directionOfTravel),
          targetPose);

      PathPlannerPath path = new PathPlannerPath(
          waypoints,
          new PathConstraints(1.0, 1.0, Math.PI, 2 * Math.PI),
          new IdealStartingState(speed, directionOfTravel),
          new GoalEndState(0.0, targetPose.getRotation().plus(Rotation2d.fromDegrees(180)))
      // if robot is not facing right way after align, maybe try rotating
      // the pose of the robot earlier
      );

      path.preventFlipping = true;

      return AutoBuilder.followPath(path);
    }, Set.of(this));
  }

  // ENSURE BLUE ALLIANCE IS SELECTED
  public Command testAlign() {
    return Commands.defer(() -> {

      Pose2d currentPose = drivetrain.getState().Pose;
      ChassisSpeeds speeds = drivetrain.getState().Speeds;

      Rotation2d directionOfTravel;
      double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

      directionOfTravel = new Rotation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
          new Pose2d(
              currentPose.getX(),
              currentPose.getY(),
              directionOfTravel),
          new Pose2d(6.5, 4, Rotation2d.fromDegrees(0)));

      PathConstraints constraints = new PathConstraints(1.0, 1.0, 1 * Math.PI, 1 * Math.PI);

      PathPlannerPath path = new PathPlannerPath(
          waypoints,
          constraints,
          new IdealStartingState(speed, directionOfTravel),
          new GoalEndState(0.0, Rotation2d.fromDegrees(0)));

      path.preventFlipping = true;

      return AutoBuilder.followPath(path);
    }, Set.of(this));
  }

  public Command printDistances() {
    return Commands.defer(() -> {
      Pose2d currentPose = drivetrain.getState().Pose;

      ChassisSpeeds speeds = drivetrain.getState().Speeds;

      Pose2d targetPose = null;
      int counter = 1;

      for (Pose2d pose : aprilTagPoses) {
        double distance = currentPose.getTranslation().getDistance(pose.getTranslation());
        System.out.println("AprilTag counter: " + counter + " Tag Pose: " + pose + "Robot Pose: " + currentPose
            + "Distance: " + distance);
        if (distance < minDistance) {
          // Unsure the idea of this
          // minDistance = distance;
          System.out.println("AprilTag counter: " + counter + " is close enough!!");
        }
        counter++;
      }

      return Commands.none();
    }, Set.of(this));
  }

  // at the end of a left/right auto align, the robot may not be at target position
  // use auto align pid, pass in targetpose and currentpose to holonomic controller
  // to calculate speeds that will be applied to drivetrain, to move robot to final destination
  // command ends when robot pose is within a tolerance of target pose
  public Command AutoAlignPID() {
    return Commands.defer(() -> {
    PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
    goalState.pose = targetPose;

    return new FunctionalCommand(
        () -> {},
        () -> {
          Pose2d currentPose = drivetrain.getState().Pose;

          drivetrain.m_pathApplyRobotSpeeds
              .withSpeeds(drivetrain.holonomicDriveController.calculateRobotRelativeSpeeds(currentPose, goalState));
        },
        null,
        () -> {
          Pose2d currentPose = drivetrain.getState().Pose;
          double positionDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
          double rotationDistance = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
          return (positionDistance < POSITION_TOLERANCE && rotationDistance < ROTATION_TOLERANCE);
        },
        drivetrain);
  }
  , Set.of(this));
}
}
