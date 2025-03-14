package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import java.util.List;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.GoalEndState;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;

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

  public Limelight() {
    limelight = NetworkTableInstance.getDefault().getTable("Limelight");
    tvSubscriber = limelight.getDoubleTopic("tv").subscribe(0.0);
    tvPublisher = limelight.getDoubleTopic("tv").publish();
    targetPoseRobotSpaceSubscriber = limelight.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
    targetPoseRobotSpacePublisher = limelight.getDoubleArrayTopic("targetpose_robotspace").publish();

    HttpCamera limelightCamera = new HttpCamera("limelight", "http://limelight.local:5800/", HttpCameraKind.kMJPGStreamer);
    CameraServer.startAutomaticCapture(limelightCamera);
  }

  @Override
  public void periodic() {
    tv = tvSubscriber.get();
    tvPublisher.set(tv);
    tx = targetPoseRobotSpaceSubscriber.get()[0];
    ty = targetPoseRobotSpaceSubscriber.get()[1];
    yaw = targetPoseRobotSpaceSubscriber.get()[4];
    targetPoseRobotSpacePublisher.set(new double[] { tx, ty, 0.0, 0.0, yaw, 0.0 });
  }

  public Command alignLeftCommand() {
    System.out.println("Called");

    if(tv == 1.0) {
      System.out.println("Sees tag");
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

      path.preventFlipping = true;

      return AutoBuilder.followPath(path);
    } else {
      return runOnce(() -> {});
    }
  }

  public Command alignRightCommand() {
    System.out.println("Called");

    if(tv == 1.0) {
      System.out.println("Sees tag");
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

      path.preventFlipping = true;
      
      return AutoBuilder.followPath(path);
    } else {
      return runOnce(() -> {});
    }
  }
}