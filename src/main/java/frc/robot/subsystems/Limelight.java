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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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


  private static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  private static final List<Pose2d> blueAprilTagPoses = new ArrayList<Pose2d>();
  private static final List<Pose2d> redAprilTagPoses = new ArrayList<Pose2d>();

  private static final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

  int counter = 0;

  Drivetrain drivetrain;
  Field2d field = new Field2d();
  NetworkTableEntry botPoseEntry;
  double[] botPose;
  public List<Pose2d> aprilTagPoses;
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
  boolean calibrate;
  public boolean autoAligning;

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

    // SmartDashboard.putData("Field", field);
    // SmartDashboard.putData(this);

    calibrate = false;
  }

  @Override
  public void periodic() {
    hasTarget = limelight.getEntry("tv").getDouble(0.0) == 1.0;

    if (hasTarget && DriverStation.isTeleop()){
      calibrate = true;
      botPose = botPoseEntry.getDoubleArray(new double[11]);
      updatedPose = new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));
      totalLatencyMs = botPose[6];
      timestamp = Timer.getFPGATimestamp() - totalLatencyMs / 1000;
      tagCount = (double) botPose[7];
      avgTagDistance = botPose[9];
                              

      xyStdDevs = 0.1 + avgTagDistance * avgTagDistance;
      rotationalStdDevs = 0.1 + avgTagDistance * avgTagDistance;

      stddevs = VecBuilder.fill(xyStdDevs, xyStdDevs, rotationalStdDevs);
      // can lower stddev if tag count is greater than 1 for funsies
      drivetrain.addVisionMeasurement(updatedPose, timestamp, stddevs);

    }


  }

  public boolean calibrate(){
    return calibrate;
  }

  public Command updateLimelight(){
    return runOnce(() -> {
      drivetrain.addVisionMeasurement(updatedPose, timestamp, stddevs);
    }
      );
  }
}
