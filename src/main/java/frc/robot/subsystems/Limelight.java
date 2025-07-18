package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private static final double OFFSET_METERS = 0.165;
  Drivetrain drivetrain;
  NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
  Field2d field = new Field2d();
  NetworkTableEntry botPoseEntry;
  double[] botPose;
  Pose2d[] aprilTagPoses;
  Pose2d[] blueAprilTagPoses;
  Pose2d[] redAprilTagPoses;
  boolean hasTarget;
  Pose2d updatedPose;
  double totalLatency;
  double timestamp;
  Pose2d currentPose;

  public Limelight(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    blueAprilTagPoses = new Pose2d[] {
      layout.getTagPose(17).get().toPose2d(),
      layout.getTagPose(18).get().toPose2d(),
      layout.getTagPose(19).get().toPose2d(),
      layout.getTagPose(20).get().toPose2d(),
      layout.getTagPose(21).get().toPose2d(),
      layout.getTagPose(22).get().toPose2d(),
    };

    redAprilTagPoses = new Pose2d[] {
      layout.getTagPose(6).get().toPose2d(),
      layout.getTagPose(7).get().toPose2d(),
      layout.getTagPose(8).get().toPose2d(),
      layout.getTagPose(9).get().toPose2d(),
      layout.getTagPose(10).get().toPose2d(),
      layout.getTagPose(11).get().toPose2d(),
    };

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if(alliance == Alliance.Blue) {
      botPoseEntry = limelight.getEntry("botpose_wpiblue");
      aprilTagPoses = blueAprilTagPoses;
    } else {
      botPoseEntry = limelight.getEntry("botpose_wpired");
      aprilTagPoses = redAprilTagPoses;
    }

    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    hasTarget = limelight.getEntry("tv").getDouble(0.0) == 1.0;

    if (hasTarget) {
      botPose = botPoseEntry.getDoubleArray(new double[11]);
      updatedPose = new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));
      totalLatency = botPose[6];
      timestamp = Timer.getFPGATimestamp() - totalLatency;
      drivetrain.addVisionMeasurement(updatedPose, timestamp);
    }
    
    field.setRobotPose(drivetrain.getState().Pose);
  }

  public Command leftAutoAlign() {
    currentPose = drivetrain.getState().Pose;

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

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      currentPose,
      targetPose
    );

    PathPlannerPath path = new PathPlannerPath(
      waypoints,
      constraints,
      null,
      new GoalEndState(0.0, Rotation2d.fromDegrees(targetPose.getRotation().getDegrees()))
    );

    path.preventFlipping = true;

    return AutoBuilder.followPath(path);
  }

  public Command rightAutoAlign() {
    currentPose = drivetrain.getState().Pose;

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

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      currentPose,
      targetPose
    );

    PathPlannerPath path = new PathPlannerPath(
      waypoints,
      constraints,
      null,
      new GoalEndState(0.0, Rotation2d.fromDegrees(targetPose.getRotation().getDegrees()))
    );

    path.preventFlipping = true;

    return AutoBuilder.followPath(path);
  }
}
