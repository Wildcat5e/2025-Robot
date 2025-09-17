// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.util.List;
// import java.util.Optional;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.Waypoint;

// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class BetterLimelightByEthan extends SubsystemBase {
//   /** Creates a new BetterLimelight. */
//   NetworkTableEntry botPoseEntry;
//   double[] botPose;
//   Pose2d currentPose;
//   Drivetrain drivetrain;
//   Field2d field = new Field2d();

//   NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
//   PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
//   double totalLatencyMs;
//   double timestamp;

//   public BetterLimelightByEthan(Drivetrain drivetrain) {
//     this.drivetrain = drivetrain;
//     SmartDashboard.putData("Field", field);

//     Optional<Alliance> alliance = DriverStation.getAlliance();
//     if (alliance.get() == Alliance.Blue) {
//       botPoseEntry = limelight.getEntry("botpose_wpiblue");
//     } else {
//       botPoseEntry = limelight.getEntry("botpose_wpired");
//     }
//   }

//   // public Command leftAutoAlign() {
//   //   List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
//   //       drivetrain.getState().Pose,
//   //       new Pose2d(6, 4, Rotation2d.fromDegrees(0)));

//   //   PathPlannerPath path = new PathPlannerPath(
//   //       waypoints,
//   //       constraints,
//   //       null,
//   //       new GoalEndState(0.0, Rotation2d.fromDegrees(0))
//   //   );

//   //   path.preventFlipping = true;

//   //   return AutoBuilder.followPath(path);
//   // }

//   @Override
//   public void periodic() {
//     if (limelight.getEntry("tv").getDouble(0) == 1) {
//       botPose = botPoseEntry.getDoubleArray(new double[11]);
//       currentPose = new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));
//       totalLatencyMs = botPose[6];
//       timestamp = Timer.getFPGATimestamp() - totalLatencyMs / 1000;
//       drivetrain.addVisionMeasurement(currentPose, timestamp);

//     }

//     field.setRobotPose(drivetrain.getState().Pose);

//     // This method will be called once per scheduler run
//   }
// }
