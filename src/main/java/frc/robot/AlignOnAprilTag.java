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

public class AlignOnAprilTag extends Command {

    public static final Rotation2d ZERO_ROTATION = Rotation2d.fromDegrees(0);
    private final PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);

    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("Limelight");
    private final DoubleArraySubscriber targetPoseRobotSpaceSubscriber =
            limelight.getDoubleArrayTopic("targetpose_robotspace")
                    .subscribe(new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    private final double xOffset;
    private final double yOffset;

    private Command followPathCommand;

    private AlignOnAprilTag(double xOffset, double yOffset) {
        this.xOffset = xOffset;
        this.yOffset = yOffset;
    }

    @Override public void initialize() {
        double[] targetPoseRobotSpace = targetPoseRobotSpaceSubscriber.get();

        double x = targetPoseRobotSpace[0] + xOffset;
        double y = targetPoseRobotSpace[1] + yOffset;
        double yaw = -targetPoseRobotSpace[4];
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(0.0, 0.0, ZERO_ROTATION),
                new Pose2d(x, y, ZERO_ROTATION)
        );

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0, Rotation2d.fromDegrees(yaw))
        );

        path.preventFlipping = true;

        followPathCommand = AutoBuilder.followPath(path);
        followPathCommand.initialize();
    }

    @Override public void execute() {
        followPathCommand.execute();
    }

    @Override public void end(boolean interrupted) {
        followPathCommand.end(interrupted);
    }

    @Override public boolean isFinished() {
        return followPathCommand.isFinished();
    }
}
