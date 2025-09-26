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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private static final double POSITION_TOLERANCE = 0.04;
  private static final double ROTATION_TOLERANCE = 0.04;

  private static final Transform2d CENTER_ALGAE_ARM = new Transform2d(
    new Translation2d(0.75, .3), 
    Rotation2d.fromDegrees(180));

  private static final Transform2d LEFT_ALIGN_DISTANCE = new Transform2d(
      new Translation2d(0.5, -0.2),
      Rotation2d.fromDegrees(180));

  private static final Transform2d RIGHT_ALIGN_DISTANCE = new Transform2d(
      new Translation2d(0.5, 0.2),
      Rotation2d.fromDegrees(180));

  double minDistance = 0.5;

  private static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  private static final List<Pose2d> blueAprilTagPoses = new ArrayList<Pose2d>();
  private static final List<Pose2d> redAprilTagPoses = new ArrayList<Pose2d>();

  private static final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

  int counter = 0;

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
  Pose2d targetPose;
  double avgTagDistance;
  double xyStdDevs;
  double rotationalStdDevs;

  public Limelight(Drivetrain drivetrain) {

    blueAprilTagPoses.add(layout.getTagPose(17).get().toPose2d());
    blueAprilTagPoses.add(layout.getTagPose(18).get().toPose2d());
    blueAprilTagPoses.add(layout.getTagPose(19).get().toPose2d());
    blueAprilTagPoses.add(layout.getTagPose(20).get().toPose2d());
    blueAprilTagPoses.add(layout.getTagPose(21).get().toPose2d());
    blueAprilTagPoses.add(layout.getTagPose(22).get().toPose2d());

    redAprilTagPoses.add(layout.getTagPose(6).get().toPose2d());
    redAprilTagPoses.add(layout.getTagPose(7).get().toPose2d());
    redAprilTagPoses.add(layout.getTagPose(8).get().toPose2d());
    redAprilTagPoses.add(layout.getTagPose(9).get().toPose2d());
    redAprilTagPoses.add(layout.getTagPose(10).get().toPose2d());
    redAprilTagPoses.add(layout.getTagPose(11).get().toPose2d());

    this.drivetrain = drivetrain;


    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    botPoseEntry = limelight.getEntry("botpose_wpiblue");
    if (alliance == Alliance.Blue) {
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
      avgTagDistance = botPose[10];


      // units in meters, every addition meter of tag distances means there is an additional 0.2 meter that the april tag is off actual location
      xyStdDevs = 0.1 + 0.2 * avgTagDistance;
      rotationalStdDevs = 0.1 + 0.1 * avgTagDistance;

      if (tagCount == 1.0) {
        stddevs = VecBuilder.fill(xyStdDevs, xyStdDevs, rotationalStdDevs);
      } 
      // can lower stddev if tag count is greater than 1 for funsies

    }

    field.setRobotPose(drivetrain.getState().Pose);

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
          System.out.println("AprilTag counter: " + counter + " is close enough!!");
        }
        counter++;
      }

      return Commands.none();
    }, Set.of(this));
  }

  public Command updateLimelight(){
    return runOnce(() -> {
      drivetrain.addVisionMeasurement(updatedPose, timestamp, stddevs);
      System.out.println(blueAprilTagPoses.get(4).transformBy(LEFT_ALIGN_DISTANCE));
    }
      );
  }

  // at the end of a left/right auto align, the robot may not be at target position
  // use auto align pid, pass in targetpose and currentpose to holonomic controller
  // to calculate speeds that will be applied to drivetrain, to move robot to final destination
  // command ends when robot pose is within a tolerance of target pose
  public Command AutoAlignPID() {
    return Commands.defer(() -> {
    
    //robotpose and currentpose are same in this command
    Pose2d robotPose = drivetrain.getState().Pose;
    Pose2d nearestTagPose = robotPose.nearest(aprilTagPoses);
    System.out.println("closest tag pose (before transform): "+ nearestTagPose);
    nearestTagPose = nearestTagPose.transformBy(LEFT_ALIGN_DISTANCE);
    System.out.println("closest tag pose (after transform): "+ nearestTagPose);
    double distance = robotPose.getTranslation().getDistance(nearestTagPose.getTranslation());
    System.out.println("robot pose: " + robotPose);
    System.out.println("distance:" + distance + "min distancne: " + minDistance);

    PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
    goalState.pose = nearestTagPose;


    if (distance < minDistance){
      System.out.println("hooray");
      return new FunctionalCommand(
        () -> {

        },
        () -> {
          drivetrain.addVisionMeasurement(updatedPose, timestamp, stddevs);

          Pose2d currentPose = drivetrain.getState().Pose;

          drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds
              .withSpeeds(drivetrain.holonomicDriveController.calculateRobotRelativeSpeeds(currentPose, goalState)));
        },
        (interrupted) -> {},
        () -> {
          Pose2d currentPose = drivetrain.getState().Pose;
          double positionDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
          double rotationDistance = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
          return (positionDistance < POSITION_TOLERANCE && rotationDistance < ROTATION_TOLERANCE);
        },
        drivetrain);
    } else {
      return Commands.none();
    }


  }
  , Set.of(drivetrain));
}

  public Command centerRobotForAlgaeArm(){
    return Commands.defer(() -> {


      //TEST AUTO ALIGN FIRST, THEN COPY PASTE THAT CODE INTO HERE
      return Commands.none();

    }, Set.of(drivetrain));
  }

}
