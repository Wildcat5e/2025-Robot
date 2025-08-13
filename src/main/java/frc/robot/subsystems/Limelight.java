package frc.robot.subsystems;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private static final double OFFSET_METERS = 0.165;
  private static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  private static final Pose2d[] blueAprilTagPoses = new Pose2d[] {
    layout.getTagPose(17).get().toPose2d(),
    layout.getTagPose(18).get().toPose2d(),
    layout.getTagPose(19).get().toPose2d(),
    layout.getTagPose(20).get().toPose2d(),
    layout.getTagPose(21).get().toPose2d(),
    layout.getTagPose(22).get().toPose2d(),
  };
  private static final Pose2d[] redAprilTagPoses = new Pose2d[] {
    layout.getTagPose(6).get().toPose2d(),
    layout.getTagPose(7).get().toPose2d(),
    layout.getTagPose(8).get().toPose2d(),
    layout.getTagPose(9).get().toPose2d(),
    layout.getTagPose(10).get().toPose2d(),
    layout.getTagPose(11).get().toPose2d(),
  };
  private static final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

  Drivetrain drivetrain;
  Field2d field = new Field2d();
  NetworkTableEntry botPoseEntry;
  double[] botPose;
  Pose2d[] aprilTagPoses;
  double[] stddevsDoubleArray;
  boolean hasTarget;
  Pose2d updatedPose;
  double totalLatencyMs;
  double timestamp;
  double tagCount;
  Matrix<N3, N1> stddevs;
  NetworkTableEntry stddevsEntry;

  public Limelight(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    stddevsEntry = limelight.getEntry("stddevs");

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if(alliance == Alliance.Blue) {
      botPoseEntry = limelight.getEntry("botpose_wpiblue");
      aprilTagPoses = blueAprilTagPoses;
    } else {
      botPoseEntry = limelight.getEntry("botpose_wpired");
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
      } else  if (tagCount > 1.0) {
        stddevs = VecBuilder.fill(stddevsDoubleArray[6], stddevsDoubleArray[7], stddevsDoubleArray[11]);
      }

      drivetrain.addVisionMeasurement(updatedPose, timestamp, stddevs);
    }
    
    field.setRobotPose(drivetrain.getState().Pose);
  }

  public Command leftAutoAlign() {
    return Commands.defer(() -> {
      Pose2d currentPose = drivetrain.getState().Pose;

      ChassisSpeeds speeds = drivetrain.getState().Speeds;

      Rotation2d directionOfTravel;
      double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vxMetersPerSecond);

      Pose2d targetPose = null;
      double minDistance = Double.MAX_VALUE;

      for (Pose2d pose : aprilTagPoses) {
        double distance = currentPose.getTranslation().getDistance(pose.getTranslation());
        if (distance < minDistance) {
          minDistance = distance;
          targetPose = new Pose2d(
            pose.getX() - OFFSET_METERS * Math.sin(pose.getRotation().getRadians()), 
            pose.getY() + OFFSET_METERS * Math.cos(pose.getRotation().getRadians()),
            pose.getRotation().plus(Rotation2d.fromDegrees(180.0))
          );
        }
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
          directionOfTravel
        ),
        targetPose
      );

      PathPlannerPath path = new PathPlannerPath(
        waypoints,
        new PathConstraints(1.0, 1.0, Math.PI, 2 * Math.PI),
        new IdealStartingState(speed, currentPose.getRotation()),
        new GoalEndState(0.0, Rotation2d.fromDegrees(targetPose.getRotation().getDegrees()))
      );

      path.preventFlipping = true;

      return AutoBuilder.followPath(path);
    }, Set.of(this));
  }

  public Command rightAutoAlign() {
    return Commands.defer(() -> {
      Pose2d currentPose = drivetrain.getState().Pose;

      ChassisSpeeds speeds = drivetrain.getState().Speeds;

      Rotation2d directionOfTravel;
      double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vxMetersPerSecond);

      Pose2d targetPose = null;
      double minDistance = Double.MAX_VALUE;

      for (Pose2d pose : aprilTagPoses) {
        double distance = currentPose.getTranslation().getDistance(pose.getTranslation());
        if (distance < minDistance) {
          minDistance = distance;
          targetPose = new Pose2d(
            pose.getX() + OFFSET_METERS * Math.sin(pose.getRotation().getRadians()), 
            pose.getY() - OFFSET_METERS * Math.cos(pose.getRotation().getRadians()),
            pose.getRotation().plus(Rotation2d.fromDegrees(180.0))
          );
        }
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
          directionOfTravel
        ),
        targetPose
      );

      PathPlannerPath path = new PathPlannerPath(
        waypoints,
        new PathConstraints(1.0, 1.0, Math.PI, 2 * Math.PI),
        new IdealStartingState(speed, currentPose.getRotation()),
        new GoalEndState(0.0, Rotation2d.fromDegrees(targetPose.getRotation().getDegrees()))
      );

      path.preventFlipping = true;

      return AutoBuilder.followPath(path);
    }, Set.of(this));
  }
}
