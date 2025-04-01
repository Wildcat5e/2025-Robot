package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

public class Vision {
    private final PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);

    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("Limelight");
    private final DoubleArraySubscriber targetPoseRobotSpaceSubscriber =
            limelight.getDoubleArrayTopic("targetpose_robotspace")
                    .subscribe(new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    private final double Y_OFFSET = 1.0;

    public Command alignLeftCommand() {
        return align(-Y_OFFSET);
    }

    public Command alignRightCommand() {
        return align(Y_OFFSET);
    }


    public Command align(double yOffset) {
        double[] targetPoseRobotSpace = targetPoseRobotSpaceSubscriber.get();

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(0, 0, Rotation2d.fromDegrees(-targetPoseRobotSpace[4])),
                new Pose2d(targetPoseRobotSpace[0], targetPoseRobotSpace[1] + yOffset, Rotation2d.fromDegrees(0))
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