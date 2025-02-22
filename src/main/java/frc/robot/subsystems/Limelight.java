package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.auto.AutoBuilder;

public class Limelight extends SubsystemBase {
  private final PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
  private NetworkTable limelight;
  private double tv;
  private DoubleSubscriber tvSubscriber;
  private DoublePublisher tvPublisher;
  private DoubleArraySubscriber targetPoseRobotSpaceSubscriber;
  private DoubleArrayPublisher targetPoseRobotSpacePublisher;
  private double tx;
  private double ty;
  private double yaw;
  private double xOffset;
  private DoubleSubscriber xOffsetSubscriber;
  private DoublePublisher xOffsetPublisher;

  public Limelight() {
    limelight = NetworkTableInstance.getDefault().getTable("Limelight");
    tvSubscriber = limelight.getDoubleTopic("tv").subscribe(0.0);
    tvPublisher = limelight.getDoubleTopic("tv").publish();
    targetPoseRobotSpaceSubscriber = limelight.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
    targetPoseRobotSpacePublisher = limelight.getDoubleArrayTopic("targetpose_robotspace").publish();
    xOffsetSubscriber = limelight.getDoubleTopic("X Offset Subscriber").subscribe(0.0);
    xOffsetPublisher = limelight.getDoubleTopic("X Offset Publisher").publish();
  }

  @Override
  public void periodic() {
    tv = tvSubscriber.get();
    tvPublisher.set(tv);
    tx = targetPoseRobotSpaceSubscriber.get()[0];
    ty = targetPoseRobotSpaceSubscriber.get()[1];
    yaw = targetPoseRobotSpaceSubscriber.get()[4];
    targetPoseRobotSpacePublisher.set(new double[] { tx, ty, 0.0, 0.0, yaw, 0.0 });
    xOffset = xOffsetSubscriber.get();
    xOffsetPublisher.set(xOffset);
  }

  public Command alignLeft() {
    if(tv == 1.0) {
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(tx, ty, Rotation2d.fromDegrees(yaw)),
        new Pose2d(tx + xOffset, ty, Rotation2d.fromDegrees(yaw))
      );

      PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null,
        new GoalEndState(0, Rotation2d.fromDegrees(yaw))
      );

      return AutoBuilder.followPath(path);
    } else {
      return runOnce(() -> {});
    }
  }

  public Command alignRight() {
    if(tv == 1.0) {
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(tx, ty, Rotation2d.fromDegrees(yaw)),
        new Pose2d(tx - xOffset, ty, Rotation2d.fromDegrees(yaw))
      );
      
      PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null,
        new GoalEndState(0, Rotation2d.fromDegrees(yaw))
      );

      return AutoBuilder.followPath(path);
    } else {
      return runOnce(() -> {});
    }
  }
}