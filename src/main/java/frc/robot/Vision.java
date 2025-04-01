package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.List;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.auto.AutoBuilder;

public class Vision {
  private final PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
  private double yOffset = 1;

  public Vision() {}

  public Command alignLeftCommand(DoubleSupplier tx, DoubleSupplier ty, DoubleSupplier yaw) {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(tx.getAsDouble(), ty.getAsDouble(), Rotation2d.fromDegrees(yaw.getAsDouble()))
    );
    
    PathPlannerPath path = new PathPlannerPath(
      waypoints,
      constraints,
      null,
      new GoalEndState(0, Rotation2d.fromDegrees(0.0))
    );

    path.preventFlipping = true;

    return AutoBuilder.followPath(path);
  }

  public Command alignRightCommand(DoubleSupplier tx, DoubleSupplier ty, DoubleSupplier yaw) {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(tx.getAsDouble(), ty.getAsDouble(), Rotation2d.fromDegrees(yaw.getAsDouble()))
    );
    
    PathPlannerPath path = new PathPlannerPath(
      waypoints,
      constraints,
      null,
      new GoalEndState(0, Rotation2d.fromDegrees(0.0))
    );

    path.preventFlipping = true;

    return AutoBuilder.followPath(path);
  }
}