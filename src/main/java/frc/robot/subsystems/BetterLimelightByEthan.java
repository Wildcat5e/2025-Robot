// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.util.Optional;

// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class BetterLimelightByEthan extends SubsystemBase {
//   /** Creates a new BetterLimelight. */
// double[] botPose;
// Pose2d currentPose;

// SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(null, null, null, currentPose)
//   NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

//   public BetterLimelightByEthan() {
//     Optional<Alliance> alliance = DriverStation.getAlliance();
//     if (alliance.get() == Alliance.Blue){
//       botPose = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[11]);
//     }
//     else {
//       botPose = limelight.getEntry("botpose_wpired").getDoubleArray(new double[11]);
//     }
//   }

//   @Override
//   public void periodic() {

//     if (limelight.getEntry("tv").getDouble(0) == 1){
//       currentPose = new Pose2d(botPose[0], botPose[1], new Rotation2d(botPose[5]));


//     }
//     // This method will be called once per scheduler run
//   }
// }
